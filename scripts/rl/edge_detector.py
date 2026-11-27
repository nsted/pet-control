"""Train the proprioceptive edge detector (docs/plans/EDGE_AVOIDANCE.md): from joint torques, tracking errors, joint velocities, head-IMU gravity direction and gyro (current tick + 0.5 s window statistics),
predict "at least one module overhangs the edge within the next 0.5 s". Episode-wise train / test split. Reports AUC, and the lead time: for each held-out episode that ends in a fall, how many seconds before the
overhang first reaches 3 modules the detector (threshold set for <= 2% false alarms per second on the pre-overhang ticks) first fires.

    python scripts/rl/edge_detector.py --data data/rl/edge/wander_data.npz --out data/rl/edge/detector.npz
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import torch

WIN = 10                                                                                                # 0.5 s feature window
LOOK = 10                                                                                               # label look-ahead in ticks (set with --look)


def features(tau: np.ndarray, err: np.ndarray, qd: np.ndarray, up: np.ndarray, gyro: np.ndarray, phase: np.ndarray, params: np.ndarray) -> np.ndarray:
    """Per-tick feature matrix for ONE episode (rows = ticks): instantaneous values + trailing-window mean / std of |tau|, err and mean of up."""
    n = len(tau)
    inst = np.concatenate([tau, err, qd * 0.3, up, gyro * 0.3, phase, params], axis=1)                       # + the CPG phase and the current gait parameters the robot itself commands
    out = [inst]
    for arr, fn in ((np.abs(tau), "mean"), (np.abs(tau), "std"), (err, "mean"), (up, "mean")):
        w = np.zeros_like(arr)
        for i in range(n):
            seg = arr[max(0, i - WIN + 1):i + 1]
            w[i] = seg.mean(axis=0) if fn == "mean" else seg.std(axis=0)
        out.append(w)
    return np.concatenate(out, axis=1)


def load(data: Path) -> tuple[list[np.ndarray], list[np.ndarray], list[np.ndarray]]:
    """Split the concatenated arrays into episodes -> (features, label y, overhang)."""
    z = np.load(data)
    lens = z["lens"]
    X, Y, O = [], [], []
    a = 0
    for L in lens:
        sl = slice(a, a + L)
        oh = z["overhang"][sl]
        y = np.array([(oh[i:i + LOOK + 1] >= 1).any() for i in range(L)], dtype=np.float32)
        X.append(features(z["tau"][sl], z["err"][sl], z["qd"][sl], z["up"][sl], z["gyro"][sl], z["phase"][sl], z["params"][sl]))
        Y.append(y)
        O.append(oh)
        a += L
    return X, Y, O


def main() -> int:
    """CLI entry point."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--look", type=int, default=10, help="label: overhang >= 1 within this many ticks (0 = overhanging now)")
    a = ap.parse_args()
    global LOOK
    LOOK = a.look
    X, Y, O = load(a.data)
    ntr = int(0.8 * len(X))
    Xtr, Ytr = np.concatenate(X[:ntr]), np.concatenate(Y[:ntr])
    mu, sd = Xtr.mean(0), Xtr.std(0) + 1e-6
    torch.manual_seed(0)
    net = torch.nn.Sequential(torch.nn.Linear(Xtr.shape[1], 64), torch.nn.ReLU(), torch.nn.Linear(64, 64), torch.nn.ReLU(), torch.nn.Linear(64, 1))
    opt = torch.optim.Adam(net.parameters(), 2e-3)
    xt, yt = torch.tensor((Xtr - mu) / sd, dtype=torch.float32), torch.tensor(Ytr)
    pw = torch.tensor((1 - Ytr.mean()) / max(Ytr.mean(), 1e-3))
    for ep in range(a.epochs):
        perm = torch.randperm(len(xt))
        for i in range(0, len(xt), 512):
            b = perm[i:i + 512]
            loss = torch.nn.functional.binary_cross_entropy_with_logits(net(xt[b]).squeeze(1), yt[b], pos_weight=pw)
            opt.zero_grad()
            loss.backward()
            opt.step()

    def score(x: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            return torch.sigmoid(net(torch.tensor((x - mu) / sd, dtype=torch.float32))).squeeze(1).numpy()

    Xte, Yte = np.concatenate(X[ntr:]), np.concatenate(Y[ntr:])
    s = score(Xte)
    order = np.argsort(s)
    ranks = np.empty(len(s))
    ranks[order] = np.arange(len(s))
    pos = Yte > 0.5
    auc = float((ranks[pos].sum() - pos.sum() * (pos.sum() - 1) / 2) / (pos.sum() * (~pos).sum()))
    # threshold: <= 2 % false alarms per second on pre-overhang ticks (label 0) => 2 % of the negative ticks
    thr = float(np.quantile(s[~pos], 0.98))
    print(f"test AUC {auc:.3f}; threshold {thr:.2f} (2% of negative ticks fire); recall on positive ticks {np.mean(s[pos] > thr):.2f}")
    leads = []
    for xe, oh in zip(X[ntr:], O[ntr:]):
        idx3 = np.where(oh >= 3)[0]
        idx1 = np.where(oh >= 1)[0]
        if len(idx1) == 0:
            continue
        se = score(xe)
        fire = np.where(se > thr)[0]
        first1 = idx1[0]
        pre = fire[fire <= (idx3[0] if len(idx3) else len(oh) - 1)]
        fires_before_commit = len(pre) > 0
        leads.append((first1 * 0.05, (pre[0] * 0.05) if fires_before_commit else float("nan"), (idx3[0] * 0.05) if len(idx3) else float("nan")))
    L = np.array(leads)
    ok = ~np.isnan(L[:, 1])
    print(f"{len(L)} held-out episodes with overhang: detector fires before 3 modules overhang in {ok.sum()}/{len(L)}; median time from first fire to 3-module overhang {np.nanmedian(L[ok, 2] - L[ok, 1]):.2f} s; "
          f"median time from first overhang tick to first fire {np.nanmedian(L[ok, 1] - L[ok, 0]):+.2f} s (negative = fires before any overhang)")
    sd_ = {k: v.detach().numpy() for k, v in net.state_dict().items()}
    np.savez(a.out, mu=mu, sd=sd, thr=thr, **sd_)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Train the selector (PPO, gamma 0.98 per 1 s step) on top of a frozen low-level run. Usage: python -m petctl.rl.hier_train LOW_RUN OUT_DIR STEPS [ENT]"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "VECLIB_MAXIMUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")


def _make(low: str, rank: int, params: dict | None = None):
    def _f():
        from stable_baselines3.common.monitor import Monitor

        from petctl.rl.hier import SelectorEnv
        e = SelectorEnv(low, params, env_run=os.environ.get("ENV_RUN"))
        e.reset(seed=1000 + rank)
        return Monitor(e)
    return _f


def main() -> None:
    """Entry point."""
    low, out, steps = sys.argv[1], Path(sys.argv[2]), int(sys.argv[3])
    ent = float(sys.argv[4]) if len(sys.argv) > 4 else 0.02
    params = json.loads(sys.argv[5]) if len(sys.argv) > 5 else None
    import stable_baselines3 as sb3
    from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize

    out.mkdir(parents=True, exist_ok=True)
    venv = VecNormalize(SubprocVecEnv([_make(low, i, params) for i in range(8)]), norm_obs=True, norm_reward=True, clip_obs=10.0)
    model = sb3.PPO("MlpPolicy", venv, n_steps=128, batch_size=256, n_epochs=8, gamma=0.98, gae_lambda=0.95, ent_coef=ent, learning_rate=lambda f: 3e-4 * f,
                    policy_kwargs={"net_arch": [128, 128]}, verbose=1, seed=0)
    model.learn(total_timesteps=steps)
    model.save(str(out / "model"))
    venv.save(str(out / "vecnormalize.pkl"))
    (out / "meta.json").write_text(json.dumps({"low": low, "steps": steps, "ent": ent}))


if __name__ == "__main__":
    main()

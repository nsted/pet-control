"""Run bookkeeping for long trainings: per-episode log, a live best-height record, periodic checkpoints and a wall-clock stop.

Files written into the run directory (all readable while training runs):

* ``episodes.jsonl``   one line per finished training episode (steps, wall time, env, every task metric)
* ``best.json``        the best height so far (record), when/where it happened, and the joint pose that achieved it
* ``status.json``      steps, wall time, episodes, best height - refreshed every checkpoint
* ``best_policy.npz``  inference policy saved shortly after each new record (rate-limited)
* ``model.zip`` / ``vecnormalize.pkl`` / ``policy.npz``  latest checkpoint

Each training session also gets a video folder ``docs/media/sessions/<run>_<timestamp>/`` with one MP4 per new local maximum (each
saved best policy that beats the previous video by ``video_min_gain_m``), named by its reward, the best marked ``BEST_`` and listed in
``index.md``. Videos render in background processes so training is not slowed.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.policy import export_policy
from petctl.rl.video import SESSIONS_DIR, finalize_session

FLAT_TOP_M = 0.0803          # highest point above the floor at the flat start (m), for reporting absolute height


class RecordCallback(BaseCallback):
    """Log episodes, track the best ``metrics["distance"]`` (height gain, m), checkpoint, and stop after ``hours``."""

    def __init__(self, out: Path, hours: float | None = None, ckpt_s: float = 900.0, best_gap_s: float = 120.0,
                 session_root: Path = SESSIONS_DIR, video_min_gain_m: float = 0.003) -> None:
        """``hours`` None = no wall-clock stop; ``ckpt_s`` checkpoint period; ``best_gap_s`` min gap between best-policy saves.

        ``session_root``/``video_min_gain_m``: where session videos go and the minimum record improvement (m) that earns a new video.
        """
        super().__init__()
        self.out, self.hours, self.ckpt_s, self.best_gap_s = out, hours, ckpt_s, best_gap_s
        self.session = session_root / f"{out.name}_{time.strftime('%Y%m%d_%H%M%S')}"
        self.video_min_gain_m = video_min_gain_m
        self._best_rec: dict[str, Any] = {}
        self._video_gain = -1.0
        self._procs: list[subprocess.Popen[bytes]] = []
        self._entries: list[dict[str, Any]] = []
        self.best = 0.0
        self.episodes = 0
        self._dirty_best = False

    def _on_training_start(self) -> None:
        self.t0 = self._last_ckpt = self._last_best_save = time.time()
        self._log = open(self.out / "episodes.jsonl", "a", buffering=1)

    def _write_best(self, m: dict[str, float], pose: dict[str, Any] | None, env: int) -> None:
        clr = "record_clearance_m" in m                      # jump task: the score IS the height of the lowest point above the floor
        top = 100 * (m["distance"] if clr else FLAT_TOP_M + m["distance"])
        rec = {"kind": "clearance" if clr else "height", "height_gain_m": m["distance"], "height_gain_cm": 100 * m["distance"], "highest_point_above_floor_cm": top,
               "timesteps": int(self.num_timesteps), "wall_s": time.time() - self.t0, "episode": self.episodes, "env": env,
               "metrics": m, "pose": pose, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        (self.out / "best.json").write_text(json.dumps(rec, indent=1))
        print(f"[record] NEW BEST {'clearance' if clr else 'height gain'} {100 * m['distance']:.1f} cm" + ("" if clr else f" (highest point {top:.1f} cm above floor)") + " "
              f"at {self.num_timesteps} steps, {(time.time() - self.t0) / 60:.0f} min", flush=True)

    def _checkpoint(self, final: bool = False) -> None:
        self.model.save(str(self.out / "model"))
        self.training_env.save(str(self.out / "vecnormalize.pkl"))
        export_policy(self.out)
        (self.out / "status.json").write_text(json.dumps({"timesteps": int(self.num_timesteps), "wall_min": (time.time() - self.t0) / 60,
                                                          "episodes": self.episodes, "best_height_gain_cm": 100 * self.best, "final": final}, indent=1))
        self._last_ckpt = time.time()

    def _save_best_policy(self) -> None:
        self.model.save(str(self.out / "best_model"))
        self.training_env.save(str(self.out / "best_vecnormalize.pkl"))
        export_policy(self.out, "best_model", "best_vecnormalize.pkl", "best_policy.npz")
        self._last_best_save, self._dirty_best = time.time(), False
        self._spawn_video()

    def _spawn_video(self) -> None:
        """Render the just-saved best policy in a background process if it is a clearly new local maximum."""
        rec = self._best_rec
        if not rec or rec["distance"] < self._video_gain + self.video_min_gain_m:
            return
        self._video_gain = rec["distance"]
        n = len(self._entries) + 1
        name = f"{n:02d}_rec{100 * rec['distance']:.1f}cm_R{rec['ret']:.0f}_ep{rec['episode']}_t{rec['wall_min']:.0f}m.mp4"
        (self.session / "policies").mkdir(parents=True, exist_ok=True)
        pol = (self.session / "policies" / name).with_suffix(".npz")
        shutil.copy(self.out / "best_policy.npz", pol)
        self._entries.append({"n": n, "video": name, "policy": str(pol.relative_to(self.session)), "record_cm": 100 * rec["distance"],
                              "episode_return": rec["ret"], "episode": rec["episode"], "timesteps": rec["timesteps"], "wall_min": rec["wall_min"]})
        (self.session / "index.json").write_text(json.dumps(self._entries, indent=1))
        log = open(self.session / "render.log", "ab")
        self._procs.append(subprocess.Popen([sys.executable, "-m", "petctl.rl.video", str(self.out), str(pol.resolve()),
                                             str(self.session / name)], stdout=log, stderr=log))

    def _on_step(self) -> bool:
        for i, (done, info) in enumerate(zip(self.locals["dones"], self.locals["infos"])):
            m = info.get("metrics")
            if not done or m is None:
                continue
            self.episodes += 1
            self._log.write(json.dumps({"timesteps": int(self.num_timesteps), "wall_s": round(time.time() - self.t0, 1), "env": i, **m}) + "\n")
            if m["distance"] > self.best:
                self.best = m["distance"]
                self._best_rec = {"distance": m["distance"], "ret": float(info.get("episode", {}).get("r", 0.0)), "episode": self.episodes,
                                  "timesteps": int(self.num_timesteps), "wall_min": (time.time() - self.t0) / 60}
                self._write_best(m, info.get("pose"), i)
                self._dirty_best = True
        now = time.time()
        if self._dirty_best and now - self._last_best_save > self.best_gap_s:
            self._save_best_policy()
        if now - self._last_ckpt > self.ckpt_s:
            self._checkpoint()
            print(f"[checkpoint] {self.num_timesteps} steps, {(now - self.t0) / 60:.0f} min, best {100 * self.best:.1f} cm", flush=True)
        return not (self.hours is not None and now - self.t0 > self.hours * 3600)

    def _on_training_end(self) -> None:
        if self._dirty_best:
            self._save_best_policy()
        self._log.close()
        for p in self._procs:                                  # let the last renders finish, then mark the best
            try:
                p.wait(timeout=180)
            except subprocess.TimeoutExpired:
                p.kill()
        best = finalize_session(self.session)
        print(f"[video] session {self.session}  best: {best}", flush=True)



class EpisodeLogCallback(BaseCallback):
    """Minimal always-on run log for tasks without a height record: ``episodes.jsonl`` (every episode's metrics) and ``status.json``.

    Same file formats as ``RecordCallback`` so ``petctl rl watch`` and the curriculum gate read any run the same way. Also checkpoints
    the model periodically so a run can be gated or inspected while it is still training, and honours an optional wall-clock stop.
    """

    def __init__(self, out: Path, hours: float | None = None, ckpt_s: float = 300.0) -> None:
        """``ckpt_s``: seconds between ``model.zip``/``policy.npz``/``status.json`` refreshes; ``hours``: optional wall-clock stop."""
        super().__init__()
        self.out, self.hours, self.ckpt_s = out, hours, ckpt_s
        self.episodes = 0

    def _on_training_start(self) -> None:
        self.t0 = self._last_ckpt = time.time()
        self._log = open(self.out / "episodes.jsonl", "a", buffering=1)

    def _write_status(self, final: bool) -> None:
        (self.out / "status.json").write_text(json.dumps({"timesteps": int(self.num_timesteps), "wall_min": (time.time() - self.t0) / 60,
                                                          "episodes": self.episodes, "updated": time.time(), "final": final}, indent=1))

    def _checkpoint(self) -> None:
        self.model.save(str(self.out / "model"))
        self.training_env.save(str(self.out / "vecnormalize.pkl"))
        export_policy(self.out)
        self._last_ckpt = time.time()
        self._write_status(False)

    def _on_step(self) -> bool:
        for i, (done, info) in enumerate(zip(self.locals["dones"], self.locals["infos"])):
            m = info.get("metrics")
            if not done or m is None:
                continue
            self.episodes += 1
            ret = float(info.get("episode", {}).get("r", float("nan")))
            self._log.write(json.dumps({"timesteps": int(self.num_timesteps), "wall_s": round(time.time() - self.t0, 1), "env": i, "return": ret, **m}) + "\n")
        if time.time() - self._last_ckpt > self.ckpt_s:
            self._checkpoint()
        return not (self.hours is not None and time.time() - self.t0 > self.hours * 3600)

    def _on_training_end(self) -> None:
        self._log.close()
        self._write_status(True)

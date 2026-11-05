"""Run bookkeeping for long trainings: per-episode log, a live best-height record, periodic checkpoints and a wall-clock stop.

Files written into the run directory (all readable while training runs):

* ``episodes.jsonl``   one line per finished training episode (steps, wall time, env, every task metric)
* ``best.json``        the best height so far (record), when/where it happened, and the joint pose that achieved it
* ``status.json``      steps, wall time, episodes, best height - refreshed every checkpoint
* ``best_policy.npz``  inference policy saved shortly after each new record (rate-limited)
* ``model.zip`` / ``vecnormalize.pkl`` / ``policy.npz``  latest checkpoint
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.policy import export_policy

FLAT_TOP_M = 0.0803          # highest point above the floor at the flat start (m), for reporting absolute height


class RecordCallback(BaseCallback):
    """Log episodes, track the best ``metrics["distance"]`` (height gain, m), checkpoint, and stop after ``hours``."""

    def __init__(self, out: Path, hours: float | None = None, ckpt_s: float = 900.0, best_gap_s: float = 120.0) -> None:
        """``hours`` None = no wall-clock stop; ``ckpt_s`` checkpoint period; ``best_gap_s`` min gap between best-policy saves."""
        super().__init__()
        self.out, self.hours, self.ckpt_s, self.best_gap_s = out, hours, ckpt_s, best_gap_s
        self.best = 0.0
        self.episodes = 0
        self._dirty_best = False

    def _on_training_start(self) -> None:
        self.t0 = self._last_ckpt = self._last_best_save = time.time()
        self._log = open(self.out / "episodes.jsonl", "a", buffering=1)

    def _write_best(self, m: dict[str, float], pose: dict[str, Any] | None, env: int) -> None:
        rec = {"height_gain_m": m["distance"], "height_gain_cm": 100 * m["distance"], "highest_point_above_floor_cm": 100 * (FLAT_TOP_M + m["distance"]),
               "timesteps": int(self.num_timesteps), "wall_s": time.time() - self.t0, "episode": self.episodes, "env": env,
               "metrics": m, "pose": pose, "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        (self.out / "best.json").write_text(json.dumps(rec, indent=1))
        print(f"[record] NEW BEST height gain {100 * m['distance']:.1f} cm (highest point {100 * (FLAT_TOP_M + m['distance']):.1f} cm above floor) "
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

    def _on_step(self) -> bool:
        for i, (done, info) in enumerate(zip(self.locals["dones"], self.locals["infos"])):
            m = info.get("metrics")
            if not done or m is None:
                continue
            self.episodes += 1
            self._log.write(json.dumps({"timesteps": int(self.num_timesteps), "wall_s": round(time.time() - self.t0, 1), "env": i, **m}) + "\n")
            if m["distance"] > self.best:
                self.best = m["distance"]
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

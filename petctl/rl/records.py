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

import collections
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.hug_buffer import CAPACITY as BUFFER_CAPACITY
from petctl.rl.hug_buffer import HugStartBuffer
from petctl.rl.hug_levels import FAMILIES, LEVEL_FILE, FamilyLevelState, LevelState
from petctl.rl.policy import export_live_policy, export_policy
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


class ProgressExportCallback(BaseCallback):
    """Every ``period_s`` of training wall time write ``progress_policy.npz`` (inference policy of the CURRENT model, same format as ``policy.npz``)
    and ``progress.json`` ``{timesteps, wall_s, rolling_success_100, time}`` into the run directory, atomically. Lets a watcher render an in-progress run.

    Cheap (a few ms, no model.zip), never consumes RNG, never raises into training. ``period_s`` defaults to ``PETCTL_PROGRESS_EXPORT_S`` or 1200 s.
    """

    def __init__(self, out: Path, period_s: float | None = None) -> None:
        """``out``: run directory; ``period_s``: seconds between exports (None: env var ``PETCTL_PROGRESS_EXPORT_S``, else 1200)."""
        super().__init__()
        self.out = out
        self.period_s = float(os.environ.get("PETCTL_PROGRESS_EXPORT_S", "1200")) if period_s is None else float(period_s)
        self._succ: collections.deque[float] = collections.deque(maxlen=100)

    def _on_training_start(self) -> None:
        self.t0 = self._last = time.time()

    def rolling_success(self) -> float | None:
        """Mean ``success`` over the last 100 finished episodes (None if the task reports no success metric or no episode finished)."""
        return float(sum(self._succ) / len(self._succ)) if self._succ else None

    def export(self) -> None:
        """Write the progress files now."""
        try:
            export_live_policy(self.model, self.training_env, self.out)
            tmp = self.out / "progress.json.tmp"
            tmp.write_text(json.dumps({"timesteps": int(self.num_timesteps), "wall_s": time.time() - self.t0, "rolling_success_100": self.rolling_success(),
                                       "time": time.time()}))
            os.replace(tmp, self.out / "progress.json")
        except Exception as e:  # noqa: BLE001 - a progress export must never stop training
            print(f"[progress-export] failed: {e}", flush=True)
        self._last = time.time()

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            m = info.get("metrics")
            if done and m is not None and "success" in m:
                self._succ.append(float(m["success"]))
        if time.time() - self._last > self.period_s:
            self.export()
        return True


class HugBufferCallback(BaseCallback):
    """Once per rollout: collect the start states every env stored (``drain_start_buffer``), merge them into the 500-state FIFO, persist the file (atomic) and broadcast the merged buffer back to
    the envs (``set_start_buffer``). The merged copy is initialised from the buffer file (resume); the file is also written at the end of training."""

    def __init__(self, path: Path | str, capacity: int = BUFFER_CAPACITY) -> None:
        """``path``: the buffer file (``$PETCTL_HUG_BUFFER``)."""
        super().__init__()
        self.buf = HugStartBuffer(path, capacity)

    def _on_step(self) -> bool:
        return True

    def _merge(self) -> None:
        new = [e for part in self.training_env.env_method("drain_start_buffer") for e in part]
        if not new:
            return
        items = self.buf.load()
        items.extend(new)
        del items[:-self.buf.capacity]
        self.buf.write_file()
        self.training_env.env_method("set_start_buffer", items)

    def _on_rollout_end(self) -> None:
        self._merge()

    def _on_training_end(self) -> None:
        self._merge()


class HugLevelCallback(BaseCallback):
    """Hug success curriculum (``petctl.rl.hug_levels``): reads each finished episode's ``hug_level`` / ``success``, advances the shared level at rolling success >= 0.30 (persisted in
    ``<out>/hug_level.json`` at once) and broadcasts the level to every env worker once per rollout (like the start buffer) and at training start (resumes from the file)."""

    def __init__(self, out: Path | str, default_level: int = 1) -> None:
        """``out``: the run directory; ``default_level``: the level when no state file exists."""
        super().__init__()
        self.path = Path(out) / LEVEL_FILE
        self.state = LevelState.load(self.path, default_level)
        self._sent = -1

    def _broadcast(self) -> None:
        if self.state.level != self._sent:
            self.training_env.env_method("set_hug_level", self.state.level)
            self._sent = self.state.level

    def _on_training_start(self) -> None:
        self._t0 = time.time()
        self.state.save(self.path)
        self.training_env.env_method("set_hug_level", self.state.level)
        self._sent = self.state.level
        print(f"[hug-level] start at level {self.state.level}", flush=True)

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            m = info.get("metrics")
            if not done or m is None or "hug_level" not in m:
                continue
            if self.state.add(int(m["hug_level"]), float(m.get("success", 0.0)), int(self.num_timesteps), time.time() - self._t0):
                self.state.save(self.path)
                print(f"[hug-level] advanced to level {self.state.level} at {self.num_timesteps} steps (rolling {self.state.history[-1]['rolling']:.3f})", flush=True)
        return True

    def _on_rollout_end(self) -> None:
        self._broadcast()
        self.state.save(self.path)


class HugFamilyLevelCallback(BaseCallback):
    """Per-family hug success curriculum (``petctl.rl.hug_levels.FamilyLevelState``, stage hug_curr_02): reads each finished episode's ``shape_kind`` / ``hug_level`` / ``success``, advances the family's
    level at rolling success >= 0.30 over its last 100 episodes (persisted in ``<out>/hug_level.json`` at once) and broadcasts the per-family levels to every env worker once per rollout and at
    training start (resumes from the file)."""

    def __init__(self, out: Path | str) -> None:
        """``out``: the run directory."""
        super().__init__()
        self.path = Path(out) / LEVEL_FILE
        self.state = FamilyLevelState.load(self.path)
        self._sent: dict[str, int] | None = None

    def _broadcast(self, force: bool = False) -> None:
        if force or self.state.levels != self._sent:
            self.training_env.env_method("set_hug_family_levels", dict(self.state.levels))
            self._sent = dict(self.state.levels)

    def _on_training_start(self) -> None:
        self._t0 = time.time()
        self.state.save(self.path)
        self._broadcast(True)
        print(f"[hug-level] start at levels {self.state.levels}", flush=True)

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            m = info.get("metrics")
            if not done or m is None or "hug_level" not in m or not 0 <= int(m.get("shape_kind", -1)) < len(FAMILIES):
                continue
            fam = FAMILIES[int(m["shape_kind"])]
            if self.state.add(fam, int(m["hug_level"]), float(m.get("success", 0.0)), int(self.num_timesteps), time.time() - self._t0):
                self.state.save(self.path)
                print(f"[hug-level] {fam} advanced to {self.state.history[-1]['level']} at {self.num_timesteps} steps (rolling {self.state.history[-1]['rolling']:.3f})", flush=True)
        return True

    def _on_rollout_end(self) -> None:
        self._broadcast()
        self.state.save(self.path)

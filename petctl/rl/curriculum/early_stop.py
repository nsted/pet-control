"""Stop a training run as soon as it passes its stage gate (checked cheaply, so training is not slowed much)."""

from __future__ import annotations

import time
from collections import deque
from pathlib import Path

from stable_baselines3.common.callbacks import BaseCallback

from petctl.rl.curriculum.gate import run_gate
from petctl.rl.curriculum.stages import Stage
from petctl.rl.policy import export_policy


class GateStopCallback(BaseCallback):
    """When the rolling training success reaches the gate's threshold (+ ``margin``), checkpoint and run the frozen gate; stop if it passes.

    The prefilter avoids running the (minutes-long) gate on policies that are clearly not ready; ``cooldown_s`` spaces repeated attempts. The margin is
    negative because training success uses the stochastic policy while the gate replays the deterministic one (S2: ~0.6 in training, 0.9 at the gate).
    """

    def __init__(self, out: Path, stage: Stage, window: int = 100, margin: float = -0.10, cooldown_s: float = 300.0) -> None:
        """``out``: run directory; ``stage``: whose gate to apply."""
        super().__init__()
        self.out, self.stage, self.margin, self.cooldown_s = out, stage, margin, cooldown_s
        self._succ: deque[float] = deque(maxlen=window)
        self._last_try = time.time() - cooldown_s      # the first attempt is not delayed
        self.passed = False

    def _on_step(self) -> bool:
        for done, info in zip(self.locals["dones"], self.locals["infos"]):
            if done and "metrics" in info:
                self._succ.append(float(info["metrics"].get("success", 0.0)))
        ready = len(self._succ) == self._succ.maxlen and sum(self._succ) / len(self._succ) >= min(1.0, self.stage.gate.min_success + self.margin)
        if not ready or time.time() - self._last_try < self.cooldown_s:
            return True
        self.model.save(str(self.out / "model"))
        self.training_env.save(str(self.out / "vecnormalize.pkl"))
        export_policy(self.out)
        res = run_gate(self.out, self.stage)
        self._last_try = time.time()
        print(f"[gate] {self.stage.name} at {self.num_timesteps} steps: {'PASS' if res['passed'] else 'fail'} "
              f"(robust success {res['robust']['success_rate']:.2f}, nominal {res['nominal']['success_rate']:.2f})", flush=True)
        self.passed = bool(res["passed"])
        return not self.passed

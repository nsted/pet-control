"""Configuration dataclasses for RL experiments. Everything an experiment varies lives here and is dumped next to each checkpoint."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any

N_JOINTS = 7
BUS_A_PER_NM2 = 13.3      # measured bus current per (N.m)^2 per motor (hanging tests, docs/plans/POWER_MANAGEMENT.md); NOT the manager's 72.5
BUS_LIMIT_A = 4.0         # battery budget (config.POWER_BUDGET), used as a soft penalty threshold in the reward


@dataclass
class DomainRand:
    """Per-episode randomisation ranges (uniform). ``scale`` in [0,1] shrinks every range towards its midpoint (curriculum)."""

    scale: float = 1.0
    delay_s: tuple[float, float] = (0.04, 0.14)
    armature: tuple[float, float] = (1e-4, 1e-3)
    kp_scale: tuple[float, float] = (0.65, 1.35)
    coulomb_scale: tuple[float, float] = (0.5, 2.0)
    cap_nm: tuple[float, float] = (0.05, 0.164)
    mass_scale: tuple[float, float] = (0.9, 1.1)
    floor_friction: tuple[float, float] = (0.4, 1.5)
    joint_offset_rad: float = 0.03
    pos_noise_rad: float = 0.002
    gyro_noise: float = 0.02
    imu_tilt_rad: float = 0.05


@dataclass
class RewardWeights:
    """Fixed reward weights (not ramped). Every term is logged separately by the env."""

    progress: float = 1.0
    energy: float = 0.5
    action_rate: float = 0.05
    joint_acc: float = 0.0005
    saturation: float = 0.05
    lateral: float = 0.3
    yaw: float = 0.1
    tilt: float = 0.5
    current: float = 1.0
    alive: float = 0.0


@dataclass
class EnvConfig:
    """Environment settings shared by training and deployment (control_hz, history, action mapping must match the real brain)."""

    control_hz: float = 20.0
    episode_s: float = 15.0
    history: int = 4                  # past actions in the observation (delay ~2 policy steps at 20 Hz)
    action_scale_rad: float = 0.5     # action in [-1,1] -> joint-angle offset about the default pose
    action_ema: float = 0.6           # target = ema*prev_target + (1-ema)*new (1.0 = frozen)
    max_target_step_rad: float = 0.35 # per-policy-step slew limit (matches the deployed protective ramp, 6 rad/s at 20 Hz = 0.3)
    pos_window: int = 3               # velocity estimate = (q_t - q_{t-w}) / (w*dt)
    flip_up_dot: float = 0.2          # terminate if body-up . world-up falls below this
    progress_window_s: float = 1.0
    seed: int = 0
    dr: DomainRand = field(default_factory=DomainRand)
    reward: RewardWeights = field(default_factory=RewardWeights)
    task: str = "locomote"

    def to_dict(self) -> dict[str, Any]:
        """Plain-dict form (JSON-serialisable) for saving with checkpoints."""
        return asdict(self)

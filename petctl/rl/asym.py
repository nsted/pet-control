"""Asymmetric actor-critic for PPO (Regimen v2 plan sec 6): the actor sees only the deployable observation, the critic additionally the privileged vector.

``PrivilegedObsWrapper`` appends ``PetEnv.privileged_vector()`` to the observation (flat, so ``VecNormalize`` handles it as usual). ``AsymActorCriticPolicy`` feeds only
the first ``n_policy`` entries to the actor MLP and all entries to the critic MLP, so ``export_policy`` can export the actor alone (``policy.npz`` is identical in
format to a symmetric run and ``GaitPolicy`` is unchanged).
"""

from __future__ import annotations

from typing import Any

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces
from stable_baselines3.common.policies import ActorCriticPolicy
from torch import nn


class PrivilegedObsWrapper(gym.Wrapper):  # type: ignore[type-arg]
    """Observation = ``[deployable obs, privileged vector]``; ``n_policy`` is the length of the first part."""

    def __init__(self, env: Any) -> None:
        """``env`` must be (or wrap) a ``PetEnv``."""
        super().__init__(env)
        self._pet = env.unwrapped
        self.n_policy = int(self._pet.observation_space.shape[0])
        self.n_priv = int(self._pet.privileged_dim())
        self.observation_space = spaces.Box(-np.inf, np.inf, (self.n_policy + self.n_priv,), np.float32)

    def _wrap(self, obs: np.ndarray) -> np.ndarray:
        return np.concatenate([obs, self._pet.privileged_vector()]).astype(np.float32)

    def reset(self, **kw: Any) -> tuple[np.ndarray, dict[str, Any]]:
        """Reset and append the privileged vector."""
        obs, info = self.env.reset(**kw)
        return self._wrap(obs), info

    def step(self, action: Any) -> tuple[np.ndarray, float, bool, bool, dict[str, Any]]:
        """Step and append the privileged vector."""
        obs, r, te, tr, info = self.env.step(action)
        return self._wrap(obs), r, te, tr, info


class AsymMlpExtractor(nn.Module):
    """Separate actor/critic MLPs; the actor reads ``features[:, :n_policy]``, the critic all of ``features``."""

    def __init__(self, feature_dim: int, n_policy: int, pi_arch: list[int], vf_arch: list[int], activation_fn: type[nn.Module], device: Any = "auto") -> None:
        """Layer layout matches ``stable_baselines3.common.torch_layers.MlpExtractor`` (``policy_net`` / ``value_net`` Sequentials)."""
        super().__init__()
        self.n_policy = n_policy

        def mlp(i: int, arch: list[int]) -> nn.Sequential:
            layers: list[nn.Module] = []
            for h in arch:
                layers += [nn.Linear(i, h), activation_fn()]
                i = h
            return nn.Sequential(*layers)

        self.policy_net = mlp(n_policy, pi_arch)
        self.value_net = mlp(feature_dim, vf_arch)
        self.latent_dim_pi = pi_arch[-1] if pi_arch else n_policy
        self.latent_dim_vf = vf_arch[-1] if vf_arch else feature_dim
        self.to(device)

    def forward(self, features: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """(actor latent, critic latent)."""
        return self.forward_actor(features), self.forward_critic(features)

    def forward_actor(self, features: torch.Tensor) -> torch.Tensor:
        """Actor latent from the deployable part only."""
        return self.policy_net(features[:, :self.n_policy])

    def forward_critic(self, features: torch.Tensor) -> torch.Tensor:
        """Critic latent from deployable + privileged."""
        return self.value_net(features)


class AsymActorCriticPolicy(ActorCriticPolicy):
    """PPO policy whose actor ignores the privileged tail of the observation. Pass ``policy_kwargs={"n_policy": N, ...}``."""

    def __init__(self, *args: Any, n_policy: int, **kwargs: Any) -> None:
        """``n_policy``: number of leading observation entries the actor may use."""
        self.n_policy = int(n_policy)
        super().__init__(*args, **kwargs)

    def _build_mlp_extractor(self) -> None:
        na = self.net_arch
        pi, vf = (na["pi"], na["vf"]) if isinstance(na, dict) else (list(na), list(na))
        self.mlp_extractor = AsymMlpExtractor(self.features_dim, self.n_policy, pi, vf, self.activation_fn, self.device)

    def _get_constructor_parameters(self) -> dict[str, Any]:
        data: dict[str, Any] = super()._get_constructor_parameters()
        data["n_policy"] = self.n_policy
        return data

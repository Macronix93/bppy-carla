from abc import ABC, abstractmethod
from typing import Optional

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from src.action_enum import *
from src.macro.macro_registry import MACRO_REGISTRY
from src.config import GLOBAL_MAX_STEPS

class BaseEnv(gym.Env, ABC):
    metadata = {"render_modes": []}

    def __init__(
        self,
        controllable_vehicles: list,
        vut_vehicle,
        action_mode: str = "micro"
    ):
        super().__init__()

        self.controllable_vehicles = controllable_vehicles
        self.vut_vehicle = vut_vehicle
        self.all_vehicles = controllable_vehicles + [vut_vehicle]

        self.action_mode = action_mode
        self.episode = 0
        self.step_count = 0
        self.max_steps = GLOBAL_MAX_STEPS

        self._setup_action_space()
        self.observation_space = spaces.Box(
            low=-1e6,
            high=1e6,
            shape=(len(self.all_vehicles), 5),
            dtype=np.float32
        )

    def _setup_action_space(self):
        if self.action_mode == "micro":
            self.action_space = spaces.MultiDiscrete([len(action_map)] * len(self.controllable_vehicles))
        elif self.action_mode == "macro":
            self.action_space = spaces.MultiDiscrete([len(action_map_macro)] * len(self.controllable_vehicles))
        else:
            raise ValueError(f"Unbekannter Action Space: {self.action_mode}")

    def _apply_action(self, action):
        for idx, veh in enumerate(self.controllable_vehicles):
            a = action[idx]

            if self.action_mode == "micro":
                self._apply_low_level_action(veh, a)
            elif self.action_mode == "macro":
                self._apply_macro_action(veh, a)

    def _apply_macro_action(self, veh, action):
        # Läuft ein Makro bereits?
        if veh.current_macro is not None:
            controller = veh.current_macro

            micro_action = controller.step()

            if controller.is_done():
                veh.current_macro = None

            self._apply_low_level_action(veh, micro_action)
            return

        # Sonst neues Makro starten
        p = veh.macro_params
        macro_factory = MACRO_REGISTRY.get(action)

        if macro_factory is not None:
            controller = macro_factory(veh, p)
            veh.current_macro = controller
            micro_action = controller.step()
        else:
            micro_action = action_map[IDLE]

        self._apply_low_level_action(veh, micro_action)

    def _get_reward(self):
        return 1.0

    def _set_seed(seed: int):
        np.random.seed(seed)

    @abstractmethod
    def reset(self, seed: Optional[int] = None, **kwargs):
        pass

    @abstractmethod
    def step(self, action):
        pass

    @abstractmethod
    def _start_simulation(self):
        pass

    @abstractmethod
    def build_vehicles(self):
        pass

    @abstractmethod
    def _apply_low_level_action(self, veh, action):
        pass

    @abstractmethod
    def _get_obs(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def render(self):
        pass

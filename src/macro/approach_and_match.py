from src.action_enum import action_map, IDLE, FASTER, SLOWER
from src.macro.macro_controller import MacroController

class ApproachAndMatchController(MacroController):
    def __init__(
        self,
        vehicle,
        in_front_vehicle,
        min_distance,
        max_distance,
        speed_eps
    ):
        super().__init__(vehicle)
        self.front = in_front_vehicle
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.speed_eps = speed_eps
        self.delta = 0.5

    def step(self):
        v = self.vehicle
        f = self.front

        # 1. Fall Behind
        if not v.is_behind(f, self.min_distance):
            if v.speed() + 2.0 > f.speed():
                return action_map[SLOWER]
            elif v.speed() - 2.0 < f.speed():
                return action_map[FASTER]

        # 2. Close Distance
        if v.is_behind(f, self.max_distance):
            if v.speed() - 2.0 < f.speed():
                return action_map[FASTER]
            elif v.speed() + 2.0 > f.speed():
                return action_map[SLOWER]

        # 3. Equalize Speeds
        if abs(v.speed() - f.speed()) <= self.speed_eps:
            return action_map[SLOWER] if v.speed() > f.speed() else action_map[FASTER]

        return action_map[IDLE]

    def is_done(self):
        v = self.vehicle
        f = self.front
        speed_diff = abs(v.speed() - f.speed())

        behind_ok = (v.is_behind(f, self.min_distance - self.delta) and
                     not v.is_behind(f, self.max_distance + self.delta))
        speed_ok = speed_diff <= self.speed_eps + self.delta
        return behind_ok and speed_ok

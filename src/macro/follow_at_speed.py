from src.action_enum import action_map, IDLE, FASTER, SLOWER
from src.macro.macro_controller import MacroController

class FollowAtSpeedController(MacroController):
    def __init__(
        self,
        vehicle,
        in_front_vehicle,
        min_distance,
        max_distance
    ):
        super().__init__(vehicle)
        self.front = in_front_vehicle
        self.min_distance = min_distance
        self.max_distance = max_distance

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

        return action_map[IDLE]

    def is_done(self):
        v = self.vehicle
        f = self.front
        return v.is_behind(f, self.min_distance) and not v.is_behind(f, self.max_distance)
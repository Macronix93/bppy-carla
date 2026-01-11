from src.action_enum import action_map, IDLE, FASTER, SLOWER
from src.macro.macro_controller import MacroController


class DriveController(MacroController):
    def __init__(self, vehicle, target_speed, speed_eps=0.05):
        super().__init__(vehicle)
        self.target_speed = target_speed
        self.speed_eps = speed_eps

    def step(self):
        v = self.vehicle
        target_speed = self.target_speed
        s = v.speed()

        if s < target_speed - self.speed_eps:
            return action_map[FASTER]

        return action_map[IDLE]

    def is_done(self):
        return self.vehicle.speed() >= self.target_speed - self.speed_eps
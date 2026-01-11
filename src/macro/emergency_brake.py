from src.action_enum import action_map, IDLE, SLOWER
from src.macro.macro_controller import MacroController


class EmergencyBrakeController(MacroController):
    def __init__(self, vehicle, brake_speed=0.0):
        super().__init__(vehicle)
        self.brake_speed = brake_speed

    def step(self):
        v = self.vehicle

        # Solange Fahrzeug noch nicht vollständig abgebremst: Langsamer werden
        if abs(v.speed() - self.brake_speed) > 0.01:
            return action_map[SLOWER]

        return action_map[IDLE]

    def is_done(self):
        return abs(self.vehicle.speed() - self.brake_speed) <= 0.01
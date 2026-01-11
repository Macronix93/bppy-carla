from src.action_enum import action_map, LANE_LEFT, LANE_RIGHT, IDLE
from src.macro.macro_controller import MacroController


class ChangeLaneController(MacroController):
    def __init__(self, vehicle, lane):
        super().__init__(vehicle)
        self.lane = lane
        self.target_lane_index = None

    def step(self):
        v = self.vehicle

        # Ziel-Lane festlegen
        if self.target_lane_index is None:
            if self.lane == "left":
                self.target_lane_index = v.lane_index() + 1
            elif self.lane == "right":
                self.target_lane_index = v.lane_index() - 1
            else:
                raise ValueError(f"Ungültige Lane: {self.lane}")

        # Auf gewünschte Lane wechseln
        if v.lane_index() < self.target_lane_index:
            return action_map[LANE_LEFT]
        elif v.lane_index() > self.target_lane_index:
            return action_map[LANE_RIGHT]

        return action_map[IDLE]

    def is_done(self):
        if self.target_lane_index is None:
            return False  # Ziel noch nicht gesetzt
        return self.vehicle.lane_index() == self.target_lane_index
from src.action_enum import action_map, LANE_LEFT, LANE_RIGHT, SLOWER, FASTER, IDLE
from src.macro.macro_controller import MacroController


class FollowVehicleController(MacroController):
    def __init__(self, vehicle, target, min_distance, max_distance):
        super().__init__(vehicle)
        self.target = target
        self.min_distance = min_distance
        self.max_distance = max_distance

    def step(self):
        v = self.vehicle
        t = self.target

        # Abstand vergrößern
        if not v.is_behind(t, self.min_distance):
            if v.speed() + 2.0 > t.speed():
                return action_map[SLOWER]
            elif v.speed() - 2.0 < t.speed():
                return action_map[FASTER]

        # Spur angleichen
        if v.lane_index() != t.lane_index():
            return action_map[LANE_LEFT] if v.lane_index() < t.lane_index() else action_map[LANE_RIGHT]

        # Abstand verkleinern
        if v.is_behind(t, self.max_distance):
            if v.speed() - 2.0 < t.speed():
                return action_map[FASTER]
            elif v.speed() + 2.0 > t.speed():
                return action_map[SLOWER]

        # Geschwindigkeit angleichen
        if abs(v.speed() - t.speed()) > 0.1:
            return action_map[SLOWER] if v.speed() > t.speed() else action_map[FASTER]

        return action_map[IDLE]

    def is_done(self):
        v = self.vehicle
        t = self.target
        delta = 0.5  # Puffer für Abstand

        lane_ok = v.lane_index() == t.lane_index()
        distance_ok = v.is_behind(t, self.min_distance - delta) and not v.is_behind(t, self.max_distance + delta)
        return lane_ok and distance_ok


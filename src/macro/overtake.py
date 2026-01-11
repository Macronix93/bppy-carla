from src.action_enum import action_map, IDLE, LANE_LEFT, LANE_RIGHT
from src.macro.drive import DriveController
from src.macro.macro_controller import MacroController


class OvertakeController(MacroController):
    def __init__(self, vehicle, target, in_front_distance, overtake_speed):
        super().__init__(vehicle)
        self.in_front_distance = in_front_distance
        self.overtake_speed = overtake_speed
        self.target = target

    def step(self):
        v = self.vehicle
        t = self.target

        # Auf Überholspur wechseln
        if v.is_behind(t, 15.0) and v.lane_index() == t.lane_index():
            return action_map[LANE_LEFT]

        # Solange nicht X Meter vorne: Beschleunigen auf bestimmte Geschwindigkeit
        if not v.is_ahead_of(t, self.in_front_distance) and v.lane_index() != t.lane_index():
            return DriveController(v, target_speed=self.overtake_speed).step()

        # Zurück auf die alte Spur wechseln
        if v.lane_index() != t.lane_index():
            return action_map[LANE_RIGHT]

        return action_map[IDLE]

    def is_done(self):
        v = self.vehicle
        t = self.target

        front_ok = v.is_ahead_of(t, self.in_front_distance)
        lane_ok = v.lane_index() == t.lane_index()
        return front_ok and lane_ok
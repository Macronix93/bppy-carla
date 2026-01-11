from src.action_enum import action_map, IDLE, LANE_RIGHT, LANE_LEFT
from src.macro.approach_and_match import ApproachAndMatchController
from src.macro.change_lane import ChangeLaneController
from src.macro.macro_controller import MacroController


class CutInController(MacroController):
    def __init__(self, vehicle, in_front_vehicle, behind_vehicle, safe_gap):
        super().__init__(vehicle)
        self.in_front_vehicle = in_front_vehicle
        self.behind_vehicle = behind_vehicle
        self.safe_gap = safe_gap

    def step(self):
        v = self.vehicle
        in_front_vehicle = self.in_front_vehicle
        behind_vehicle = self.behind_vehicle
        safe_gap = self.safe_gap

        if v.is_behind(in_front_vehicle, safe_gap) and v.is_ahead_of(behind_vehicle, safe_gap):
            if v.lane_index() > in_front_vehicle.lane_index():
                return ChangeLaneController(v, "right").step()
            else:
                return ChangeLaneController(v, "left").step()
        else:
            return ApproachAndMatchController(v, in_front_vehicle, safe_gap, safe_gap, 0.1).step()

    def is_done(self):
        v = self.vehicle
        f = self.in_front_vehicle
        b = self.behind_vehicle
        delta = 0.5
        safe_gap = self.safe_gap

        lane_ok = v.lane_index() == f.lane_index()
        between_ok = v.is_behind(f, safe_gap - delta) and v.is_ahead_of(b, safe_gap - delta)
        return lane_ok and between_ok

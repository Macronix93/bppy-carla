from src.action_enum import action_map_macro, FOLLOW_VEHICLE, DRIVE, OVERTAKE, CUT_IN, APPROACH_AND_MATCH, \
    FOLLOW_AT_SPEED, EMERGENCY_BRAKE, CHANGE_LANE
from src.macro.approach_and_match import ApproachAndMatchController
from src.macro.change_lane import ChangeLaneController
from src.macro.cut_in import CutInController
from src.macro.drive import DriveController
from src.macro.emergency_brake import EmergencyBrakeController
from src.macro.follow_at_speed import FollowAtSpeedController
from src.macro.follow_vehicle import FollowVehicleController
from src.macro.overtake import OvertakeController

MACRO_REGISTRY = {
    action_map_macro[FOLLOW_VEHICLE]: lambda veh, p: FollowVehicleController(
        veh, p["target"], p.get("min_distance", 20.0), p.get("max_distance", 20.0)
    ),
    action_map_macro[DRIVE]: lambda veh, p: DriveController(
        veh, p.get("target_speed", 10.0), p.get("speed_eps", 0.05)
    ),
    action_map_macro[OVERTAKE]: lambda veh, p: OvertakeController(
        veh, p["target"], p.get("in_front_distance", 15.0), p.get("overtake_speed", 20.0)
    ),
    action_map_macro[CUT_IN]: lambda veh, p: CutInController(
        veh, p["in_front_vehicle"], p["behind_vehicle"], p.get("safe_gap", 15.0)
    ),
    action_map_macro[APPROACH_AND_MATCH]: lambda veh, p: ApproachAndMatchController(
        veh, p["in_front_vehicle"], p.get("min_distance", 20.0), p.get("max_distance", 20.0), p.get("speed_eps", 0.1)
    ),
    action_map_macro[FOLLOW_AT_SPEED]: lambda veh, p: FollowAtSpeedController(
        veh, p["in_front_vehicle"], p.get("min_distance", 20.0), p.get("max_distance", 20.0)
    ),
    action_map_macro[EMERGENCY_BRAKE]: lambda veh, p: EmergencyBrakeController(
        veh, p.get("brake_speed", 0.0)
    ),
    action_map_macro[CHANGE_LANE]: lambda veh, p: ChangeLaneController(
        veh, p.get("target_lane", "left")
    ),
}

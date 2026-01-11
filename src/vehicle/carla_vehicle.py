import math
from typing import Optional, Tuple

import carla

from src.agents.navigation.local_planner import RoadOption
from src.config import CARLA_DEBUG
from src.vehicle.base_vehicle import BaseVehicle, BaseControllableVehicle


class CarlaVehicle(BaseVehicle):
    """
    Klasse, die ein Fahrzeug in CARLA kapselt.
    """
    def __init__(self, vehicle_id: str, carla_blueprint_id: str, spawn_index: Optional[int] = None,
                 vehicle_color: Tuple[int, int, int] = (0, 0, 0), initial_speed: Optional[float] = 10.0,
                 tm_settings: Optional[dict] = None
                 ):
        super().__init__(vehicle_id)
        self.vehicle_id = vehicle_id
        self.carla_blueprint: Optional[carla.Blueprint] = None
        self.carla_blueprint_id = carla_blueprint_id
        self.spawn_index = spawn_index
        self.spawn_point: Optional[carla.Transform] = None
        self.vehicle_color = vehicle_color
        self.carla_actor: Optional[carla.Actor] = None
        self.initial_speed = initial_speed
        self.tm_settings = tm_settings

    def speed(self) -> float:
        return self.carla_actor.get_velocity().length() # in m/s

    def get_location(self):
        return self.carla_actor.get_transform().location

    def lane_index(self):
        # Den aktuellen Waypoint des Fahrzeugs holen
        waypoint = self.carla_actor.get_world().get_map().get_waypoint(
            self.carla_actor.get_location(), project_to_road=True, lane_type=carla.LaneType.Driving
        )

        if not waypoint:
            return 999  # Fallback

        # In CARLA beginnt die Zählung von der Mitte nach außen (typischerweise 1, 2, 3...)
        return 3 - abs(waypoint.lane_id) # 3 - ... um auf SUMO abzubilden

    def is_behind(self, other_vehicle, threshold=0.0):
        loc_self = self.get_location()
        loc_other = other_vehicle.get_location()

        # Projektion auf Straße / Forward Vektor des anderen Fahrzeugs
        f = other_vehicle.carla_actor.get_transform().get_forward_vector()
        rel = (loc_self.x - loc_other.x) * f.x + (loc_self.y - loc_other.y) * f.y

        return rel < -threshold

    def is_ahead_of(self, other_vehicle, threshold=0.0):
        loc_self = self.get_location()
        loc_other = other_vehicle.get_location()

        # Forward Vektor des anderen Fahrzeugs
        f = other_vehicle.carla_actor.get_transform().get_forward_vector()
        rel = (loc_self.x - loc_other.x) * f.x + (loc_self.y - loc_other.y) * f.y

        return rel > threshold

    def distance_to(self, other_vehicle):
        loc1 = self.carla_actor.get_location()
        loc2 = other_vehicle.carla_actor.get_location()

        dx = loc1.x - loc2.x
        dy = loc1.y - loc2.y
        dz = loc1.z - loc2.z

        return (dx * dx + dy * dy + dz * dz) ** 0.5

    def spectate(self, mode="topdown"):
        veh = self.carla_actor
        tf = veh.get_transform()
        spec = veh.get_world().get_spectator()

        if mode == "topdown":
            spec.set_transform(carla.Transform(
                carla.Location(tf.location.x, tf.location.y, tf.location.z + 50),
                carla.Rotation(pitch=-90)
            ))
            return

        if mode == "thirdperson":
            yaw = math.radians(tf.rotation.yaw)
            dx = -8 * math.cos(yaw)
            dy = -8 * math.sin(yaw)

            spec.set_transform(carla.Transform(
                carla.Location(tf.location.x + dx, tf.location.y + dy, tf.location.z + 3),
                carla.Rotation(pitch=-10, yaw=tf.rotation.yaw)
            ))
            return

        print(f"Unbekannter Modus: {mode}")


class CarlaControllableVehicle(CarlaVehicle, BaseControllableVehicle):
    def __init__(self, *args, vehicle_smt_var=None, lp_opts=None, **kwargs):
        CarlaVehicle.__init__(self, *args, **kwargs)
        BaseControllableVehicle.__init__(self, vehicle_id=args[0], vehicle_smt_var=vehicle_smt_var)
        self.lane_change_requested = None
        self.lane_change_active = False
        self.lp_opts = lp_opts if lp_opts is not None else {}
        self.planners = {}

    # ----------------------------------------
    # Lane-Change API
    # ----------------------------------------
    def request_lane_change(self, direction: str):
        """
        Fordert einen Spurwechsel an ("left" oder "right").
        Wird nur gesetzt, wenn kein Spurwechsel aktiv ist.
        """
        if not self.lane_change_active and self.lane_change_requested is None:
            if direction in ("left", "right"):
                self.lane_change_requested = direction

    def perform_lane_change_step(self, planner):
        """
        Führt den Spurwechsel aus, wenn angefragt.
        Muss bei jedem Control-Step aufgerufen werden.
        """
        if self.lane_change_requested is not None and not self.lane_change_active:
            # Aktuellen Waypoint holen
            current_wp = self.carla_actor.get_world().get_map().get_waypoint(
                self.carla_actor.get_location(),
                project_to_road=True,
                lane_type=carla.LaneType.Driving
            )

            if current_wp:
                plan = self.generate_lane_change_plan(
                    start_waypoint=current_wp,
                    direction=self.lane_change_requested,
                    distance_same_lane=6.0,
                    distance_other_lane=18.0,
                    lane_change_forward=8.0,
                )

                if len(plan) > 0:
                    planner.set_global_plan(plan, stop_waypoint_creation=True, clean_queue=True)
                    self.lane_change_active = True
                else:
                    print(f"{self.vehicle_id}: Spurwechsel nicht möglich.")
                    self.lane_change_requested = None

        # Local Planner Step
        control = planner.run_step(debug=CARLA_DEBUG)
        self.carla_actor.apply_control(control)

        # Lane-Change fertig?
        if self.lane_change_active and planner.done():
            planner._stop_waypoint_creation = False
            safe_wp = self.carla_actor.get_world().get_map().get_waypoint(self.carla_actor.get_location())
            planner._waypoints_queue.append((safe_wp, RoadOption.LANEFOLLOW))

            self.lane_change_requested = None
            self.lane_change_active = False
            self.carla_actor.set_light_state(carla.VehicleLightState.NONE)

    def generate_lane_change_plan(self,
                                  start_waypoint,
                                  direction='left',
                                  distance_same_lane=10.0,
                                  distance_other_lane=25.0,
                                  lane_change_forward=8.0,
                                  lane_changes=1,
                                  step_distance=2.0,
                                  check=True):
        """
        Generate a list[(carla.Waypoint, RoadOption)] that describes a lane change maneuver.

        Parameters tuned to be conservative:
          - distance_same_lane: distance to keep in current lane before initiating lane change
          - lane_change_forward: how far ahead to evaluate the lane-change point
          - lane_changes: how many lane moves (1 = adjacent lane)
        """
        distance_same_lane = max(distance_same_lane, 0.1)
        distance_other_lane = max(distance_other_lane, 0.1)
        lane_change_forward = max(lane_change_forward, 0.1)

        plan = []
        plan.append((start_waypoint, RoadOption.LANEFOLLOW))

        # accumulate forward a bit on the same lane
        distance = 0.0
        while distance < distance_same_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                return []  # can't extend
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        # Decide lane change option
        if direction.lower() == 'left':
            option = RoadOption.CHANGELANELEFT
        elif direction.lower() == 'right':
            option = RoadOption.CHANGELANERIGHT
        else:
            return []

        lane_changes_done = 0
        # allow dividing the lane-change across multiple small forward moves
        lane_change_segment = lane_change_forward / max(1, lane_changes)

        while lane_changes_done < lane_changes:
            # Move forward to the lane-change point
            next_wps = plan[-1][0].next(lane_change_segment)
            if not next_wps:
                return []
            next_wp = next_wps[0]

            # Get side lane
            if direction.lower() == 'left':
                if check and str(next_wp.lane_change) not in ['Left', 'Both']:
                    return []
                side_wp = next_wp.get_left_lane()
            else:
                if check and str(next_wp.lane_change) not in ['Right', 'Both']:
                    return []
                side_wp = next_wp.get_right_lane()

            if not side_wp or side_wp.lane_type != carla.LaneType.Driving:
                return []

            # Append the lane-change waypoint (mark its option)
            plan.append((side_wp, option))
            lane_changes_done += 1

        # After lane change, travel a bit in the other lane (stabilize)
        distance = 0.0
        while distance < distance_other_lane:
            next_wps = plan[-1][0].next(step_distance)
            if not next_wps:
                break
            next_wp = next_wps[0]
            distance += next_wp.transform.location.distance(plan[-1][0].transform.location)
            plan.append((next_wp, RoadOption.LANEFOLLOW))

        return plan
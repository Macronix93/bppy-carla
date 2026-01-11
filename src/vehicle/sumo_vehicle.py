from typing import Tuple

import traci

from src.vehicle.base_vehicle import BaseVehicle, BaseControllableVehicle


class SumoVehicle(BaseVehicle):
    """
    A class representing a controllable vehicle in SUMO.
    Inherits from SumoVehicle and adds methods for controlling the vehicle.
    """

    def __init__(self, vehicle_id: str, route_edges: str = ["entry", "longEdge", "exit", "exit2"], typeID: str = "manual",
                    depart_time: float = 0, depart_pos: float = 0.0, depart_lane: int = 0, depart_speed: float | str | None = None,
                    vehicle_color: Tuple[int, int, int] = (0, 255, 0), lane_change_mode: int | None = None,
                    speed_mode: int | None = None, current_speed: float | None = None
        ):

        super().__init__(vehicle_id)
        self.vehicle_id = vehicle_id
        self.route_edges = route_edges
        self.typeID = typeID
        self.depart_time = depart_time
        self.depart_pos = depart_pos
        self.depart_lane = depart_lane
        self.depart_speed = depart_speed
        self.vehicle_color = vehicle_color
        self.lane_change_mode = lane_change_mode
        self.speed_mode = speed_mode
        self.current_speed = current_speed

    def speed(self):
        """
        Get the current speed of the vehicle.

        Returns:
            float: The current speed of the vehicle in m/s.
        """
        return traci.vehicle.getSpeed(self.vehicle_id)

    def lane_index(self):
        """
        Get the current lane index of the vehicle.

        Returns:
            int: The index of the lane the vehicle is currently in.
        """
        return traci.vehicle.getLaneIndex(self.vehicle_id)

    def is_behind(self, other_vehicle, threshold=0.0):
        """
        Check if the vehicle is behind another vehicle by a certain threshold.
        :param other_vehicle: id of the other vehicle to check against
        :param threshold: a distance threshold in meters
        :return: true if this vehicle is behind the other vehicle by more than the threshold
        """
        x_self, _ = traci.vehicle.getPosition(self.vehicle_id)
        x_other, _ = traci.vehicle.getPosition(other_vehicle.vehicle_id)

        return x_self < x_other - threshold

    def is_ahead_of(self, other_vehicle, threshold=0.0):
        x_self, _ = traci.vehicle.getPosition(self.vehicle_id)
        x_other, _ = traci.vehicle.getPosition(other_vehicle.vehicle_id)

        return x_self > x_other + threshold

    def distance_to(self, other_vehicle):
        x1, y1 = traci.vehicle.getPosition(self.vehicle_id)
        x2, y2 = traci.vehicle.getPosition(other_vehicle.vehicle_id)

        dx = x1 - x2
        dy = y1 - y2

        return (dx * dx + dy * dy) ** 0.5


class SumoControllableVehicle(SumoVehicle, BaseControllableVehicle):
    def __init__(self, *args, vehicle_smt_var=None, **kwargs):
        SumoVehicle.__init__(self, *args, **kwargs)
        BaseControllableVehicle.__init__(self, vehicle_id=args[0], vehicle_smt_var=vehicle_smt_var)


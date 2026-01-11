from abc import ABC, abstractmethod

from src.action_enum import *


class BaseVehicle(ABC):
    def __init__(self, vehicle_id: str):
        self.vehicle_id = vehicle_id

    @abstractmethod
    def speed(self) -> float:
        pass

    @abstractmethod
    def lane_index(self) -> int:
        pass

    @abstractmethod
    def is_behind(self, other_vehicle, threshold: float) -> bool:
        pass

    @abstractmethod
    def is_ahead_of(self, other_vehicle, threshold: float) -> bool:
        pass

    @abstractmethod
    def distance_to(self, other_vehicle) -> float:
        pass


class BaseControllableVehicle(BaseVehicle, ABC):
    def __init__(self, vehicle_id: str, vehicle_smt_var=None):
        super().__init__(vehicle_id)
        self.vehicle_smt_var = vehicle_smt_var
        self.macro_params = {}
        self.current_macro = None

    def set_macro_params(self, **kwargs):
        """
        Setzt Parameter für den Makro-Controller.
        Beispiel: v1.set_macro_params(target=vut, min_distance=20, max_distance=30)
        """
        self.macro_params.update(kwargs)

    # --- LOW LEVEL ACTIONS ---
    def LANE_LEFT(self):
        return self.vehicle_smt_var == LANE_LEFT

    def LANE_RIGHT(self):
        return self.vehicle_smt_var == LANE_RIGHT

    def FASTER(self):
        return self.vehicle_smt_var == FASTER

    def SLOWER(self):
        return self.vehicle_smt_var == SLOWER

    def IDLE(self):
        return self.vehicle_smt_var == IDLE

    # --- MACRO ACTIONS ---
    def FOLLOW_VEHICLE(self):
        return self.vehicle_smt_var == FOLLOW_VEHICLE

    def DRIVE(self):
        return self.vehicle_smt_var == DRIVE

    def OVERTAKE(self):
        return self.vehicle_smt_var == OVERTAKE

    def CUT_IN(self):
        return self.vehicle_smt_var == CUT_IN

    def APPROACH_AND_MATCH(self):
        return self.vehicle_smt_var == APPROACH_AND_MATCH

    def FOLLOW_AT_SPEED(self):
        return self.vehicle_smt_var == FOLLOW_AT_SPEED

    def EMERGENCY_BRAKE(self):
        return self.vehicle_smt_var == EMERGENCY_BRAKE

    def CHANGE_LANE(self):
        return self.vehicle_smt_var == CHANGE_LANE
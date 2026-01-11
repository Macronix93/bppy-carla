from abc import ABC

from src.vehicle.base_vehicle import BaseVehicle, BaseControllableVehicle


class HighwayVehicle(BaseVehicle):

    def __init__(self, v_index, vehicle_id: str):
        super().__init__(vehicle_id)
        self.env = None
        self.v_index = v_index
        self.env_vehicle = None
        self.vehicle_id = vehicle_id

    def delta_pos(self, other_vehicle):
        return self.env_vehicle.position - other_vehicle.env_vehicle.position

    def delta_x_pos(self, other_vehicle):
        return self.env_vehicle.position[0] - other_vehicle.env_vehicle.position[0]

    def is_ahead_of(self, other_vehicle, more_than=0.0):
        # TODO: Only works when traveling in x-direction, needs to incorporate heading etc.
        return self.env_vehicle.position[0] - more_than > other_vehicle.env_vehicle.position[0]

    def is_behind(self, other_vehicle, more_than=0.0):
        # TODO: Only works when traveling in x-direction, needs to incorporate heading etc.
        return self.env_vehicle.position[0] + more_than < other_vehicle.env_vehicle.position[0]

    def speed(self):
        return self.env_vehicle.speed

    def target_speed(self):
        return self.env_vehicle.target_speed

    def velocity(self):
        return self.env_vehicle.velocity

    def lane_index(self):
        # Highway-Env: linke Spur = 0, rechte Spur = max
        current_index = self.env_vehicle.lane_index[-1]  # z.B. ('0','1',2)
        max_index = self.env.unwrapped.config['lanes_count'] - 1
        # Umkehren: linke Spur -> max_index, rechte Spur -> 0
        return max_index - current_index

    def set_env(self, env):
        self.env = env
        self.env_vehicle = self.env.unwrapped.road.vehicles[self.v_index]

    def distance_to(self, other_vehicle):
        pass

class HighwayControllableVehicle(HighwayVehicle, BaseControllableVehicle):

    def __init__(self, v_index, vehicle_id, vehicle_smt_var=None):
        HighwayVehicle.__init__(self, v_index=v_index, vehicle_id=vehicle_id)
        BaseControllableVehicle.__init__(self, vehicle_id=vehicle_id, vehicle_smt_var=vehicle_smt_var)

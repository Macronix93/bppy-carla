from src.vehicle.base_vehicle import BaseControllableVehicle


class MacroController:
    def __init__(self, vehicle: BaseControllableVehicle):
        self.vehicle = vehicle

    def step(self) -> int:
        raise NotImplementedError

    def is_done(self) -> bool:
        pass

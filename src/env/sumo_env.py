import os
import sys
from typing import Optional

import numpy as np
import traci

from src.env.base_env import BaseEnv
from src.config import SUMO_DELAY, SUMO_COLLISION_ACTION, SUMO_STEP_LENGTH, SUMO_TIME_TO_TELEPORT, \
    SUMO_LATERAL_RESOLUTION
from src.vehicle.sumo_vehicle import SumoVehicle, SumoControllableVehicle


class SumoEnv(BaseEnv):
    def __init__(self, sumo_config_file: str, controllable_vehicles: list[SumoControllableVehicle],
                 vut_vehicle: SumoVehicle, action_mode="micro"):
        super().__init__(controllable_vehicles, vut_vehicle, action_mode)

        self.sumo_config_file = sumo_config_file

    # ------------------------------------------------------------------
    # RESET, STEP
    # ------------------------------------------------------------------

    def reset(self, seed: Optional[int] = None, **kwargs):
        if self.episode != 0:
            self.close()

        self.episode += 1
        self.step_count = 0

        if seed is not None:
            self.sumo_seed = seed

        self._start_simulation()
        self.build_vehicles()

        # Wait for the simulation to start, finish setting up vehicles, etc.
        traci.simulationStep()

    def step(self, action):
        traci.gui.trackVehicle("View #0", "veh_manual_1")

        collisions = traci.simulation.getCollisions()
        if collisions:
            print("Collision detected! Exiting simulation...")
            traci.close()
            raise SystemExit()

        self._apply_action(action)
        traci.simulationStep()
        self.step_count += 1

        obs = self._get_obs()
        reward = self._get_reward()
        terminated = bool(self.step_count >= self.max_steps or len(collisions) > 0)
        truncated = False

        return obs, reward, terminated, truncated, {}

    # ------------------------------------------------------------------
    # SIMULATION SETUP
    # ------------------------------------------------------------------

    def _start_simulation(self, sumo_gui: bool = True):
        """
        Setup and start the SUMO-traci connection.

        Args:
            config_path (str): Path to the .sumocfg file, to set up the simulation environment.
            sumo_gui (bool): Whether to run sumo-gui or just sumo in command line.
        """
        # Check SUMO_HOME environment
        if "SUMO_HOME" not in os.environ:
            sys.exit("Please declare environment variable 'SUMO_HOME'")

        sumo_bin = "sumo-gui" if sumo_gui else "sumo"
        sumo_bin += ".exe" if sys.platform == "win32" else ""
        sumo_bin_path = os.path.join(os.environ["SUMO_HOME"], "bin", sumo_bin)

        if not os.path.isfile(sumo_bin_path):
            sys.exit(f"SUMO executable not found at: {sumo_bin_path}")

        # Append tools path for traci import
        tools = os.path.join(os.environ["SUMO_HOME"], "tools")
        sys.path.append(tools)

        sumo_config = [
            sumo_bin_path,
            "-c",
            self.sumo_config_file,
            "--step-length",
            str(SUMO_STEP_LENGTH),
            "--delay",
            SUMO_DELAY,
            "--lateral-resolution",
            str(SUMO_LATERAL_RESOLUTION),
            "--collision.action",
            SUMO_COLLISION_ACTION,
            "--time-to-teleport",
            SUMO_TIME_TO_TELEPORT,
            "--start",
            "--quit-on-end",
            "--log",
            "D:/Program Files (x86)/Eclipse/Sumo/logfile.txt"
        ]

        traci.start(sumo_config)
        traci.gui.setZoom("View #0", 1200)
        traci.gui.setOffset("View #0", -100, -196)

    # ------------------------------------------------------------------
    # VEHICLES
    # ------------------------------------------------------------------

    def build_vehicles(self):
        """
        Build and add vehicles to the simulation.
        :return:
        """
        for vehicle in self.all_vehicles:

            traci.vehicle.addFull(
                vehID=vehicle.vehicle_id,
                routeID="",
                typeID=vehicle.typeID,
                depart=vehicle.depart_time,
                departPos=vehicle.depart_pos,
                departLane=vehicle.depart_lane,
                departSpeed=vehicle.depart_speed
                if vehicle.depart_speed is not None else "avg",
            )

            traci.vehicle.setRoute(vehicle.vehicle_id, vehicle.route_edges)
            traci.vehicle.setColor(vehicle.vehicle_id, vehicle.vehicle_color)

            if vehicle.speed_mode is not None:
                traci.vehicle.setSpeedMode(vehicle.vehicle_id, vehicle.speed_mode)

            if vehicle.current_speed is not None:
                traci.vehicle.setSpeed(vehicle.vehicle_id, vehicle.current_speed)

            if vehicle.lane_change_mode is not None:
                traci.vehicle.setLaneChangeMode(
                    vehicle.vehicle_id,
                    vehicle.lane_change_mode
                )

    # ------------------------------------------------------------------
    # LOW LEVEL ACTION
    # ------------------------------------------------------------------

    def _apply_low_level_action(self, vehicle, action):
        vehicle_id = vehicle.vehicle_id
        speed = traci.vehicle.getSpeed(vehicle_id)
        if action == 0:
            if 0 <= traci.vehicle.getLaneIndex(vehicle_id) + 1 < 3:
                traci.vehicle.changeLane(
                    vehicle_id, traci.vehicle.getLaneIndex(vehicle_id) + 1, 1
                )
        elif action == 1:
            traci.vehicle.setSpeed(
                vehicle_id, speed
            )  # Idle action, maintain current speed
        elif action == 2:
            if 0 <= traci.vehicle.getLaneIndex(vehicle_id) - 1 < 3:
                traci.vehicle.changeLane(
                    vehicle_id, traci.vehicle.getLaneIndex(vehicle_id) - 1, 1
                )
        elif action == 3:
            traci.vehicle.setSpeed(vehicle_id, speed + 1)
        elif action == 4:
            traci.vehicle.setSpeed(vehicle_id, max(0, speed - 1))

    # ------------------------------------------------------------------
    # OBSERVATION / TERMINATION
    # ------------------------------------------------------------------

    def _get_obs(self) -> np.ndarray:
        obs = []

        for veh_id in traci.vehicle.getIDList():
            try:
                speed = traci.vehicle.getSpeed(veh_id)
                position = traci.vehicle.getPosition(veh_id)

                # Optional: include ID hash or vehicle role
                obs.append([speed, *position])

            except traci.TraCIException:
                obs.append([0.0, 0.0, 0.0, 0.0])  # or np.zeros(4)

        return np.array(obs, dtype=np.float32)

    def get_lane_number(lane_id: str) -> Optional[int]:
        import re

        match = re.search(r"_(\d+)$", lane_id)
        return int(match.group(1)) if match else None

    def _get_info(self):
        # TODO: Implement info logic
        pass

    def render(self):
        # TODO: Implement rendering logic
        pass

    def _render_frame(self):
        # TODO: Implement frame rendering logic
        pass

    # ------------------------------------------------------------------
    # CLEANUP
    # ------------------------------------------------------------------

    def close(self):
        traci.close()
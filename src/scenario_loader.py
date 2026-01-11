import json
from pathlib import Path

import gymnasium as gym
import numpy as np
from z3 import *

from src.action_enum import Actions, MacroActions
from src.config import CARLA_HOST, CARLA_PORT, CURRENT_SIM, ACTION_SPACE, HIGHWAY_SIMULATION_FREQUENCY, \
    HIGHWAY_POLICY_FREQUENCY
from src.vehicle.carla_vehicle import CarlaVehicle, CarlaControllableVehicle
from src.vehicle.highway_vehicle import HighwayControllableVehicle, HighwayVehicle
from src.vehicle.sumo_vehicle import SumoVehicle, SumoControllableVehicle
from src.register_env import *
import highway_env

class ScenarioLoader:
    def load_scenario(self, scenario_name):
        # Lade Szenario Config JSON
        scenario_dir = Path(__file__).parent / "scenario"
        scenario_path = scenario_dir / f"{scenario_name}.json"

        with open(scenario_path, "r") as f:
            cfg = json.load(f)

        # Lade Fahrzeugdaten aus JSON
        vehicles = {}

        # Simulation Seed
        env_seed = cfg.get("seed", None)

        for name, vcfg in cfg["vehicles"].items():
            color = tuple(vcfg["color"])

            if "smt_var" in vcfg:
                action_sort = Actions if ACTION_SPACE == "micro" else MacroActions
                smt_var = Const(vcfg["smt_var"], action_sort)
            else:
                smt_var = None

            if CURRENT_SIM == "carla":
                c = vcfg["carla"]

                if vcfg["class"] == "controllable":
                    vehicles[name] = CarlaControllableVehicle(
                        name,
                        carla_blueprint_id=c["blueprint"],
                        spawn_index=c["spawn_index"],
                        vehicle_color=color,
                        initial_speed=c["initial_speed"],
                        lp_opts=c.get("lp_opts", None),
                        vehicle_smt_var=smt_var
                    )
                else:
                    vehicles[name] = CarlaVehicle(
                        name,
                        carla_blueprint_id=c["blueprint"],
                        spawn_index=c["spawn_index"],
                        initial_speed=c["initial_speed"],
                        tm_settings=c.get("tm_settings", None),
                        vehicle_color=color
                    )

            elif CURRENT_SIM == "sumo":
                s = vcfg["sumo"]
                depart_speed = s["depart_speed"]

                # depart_speed kann bei SUMO ein Float-Wert oder ein String sein
                if isinstance(depart_speed, str):
                    try:
                        depart_speed = float(depart_speed)
                    except ValueError:
                        pass

                if vcfg["class"] == "controllable":
                    vehicles[name] = SumoControllableVehicle(
                        name,
                        route_edges=s["route"],
                        typeID=s["typeID"],
                        depart_time=s["depart_time"],
                        depart_pos=s["depart_pos"],
                        depart_lane=s["depart_lane"],
                        depart_speed=depart_speed,
                        vehicle_color=color,
                        lane_change_mode=s["lane_change_mode"],
                        speed_mode=s["speed_mode"],
                        vehicle_smt_var=smt_var
                    )
                else:
                    vehicles[name] = SumoVehicle(
                        name,
                        route_edges=s["route"],
                        typeID=s["typeID"],
                        depart_time=s["depart_time"],
                        depart_pos=s["depart_pos"],
                        depart_lane=s["depart_lane"],
                        depart_speed=depart_speed,
                        lane_change_mode=s.get("lane_change_mode", None),
                        speed_mode=s.get("speed_mode", None),
                        current_speed=s.get("speed", None),
                        vehicle_color=color
                    )

            elif CURRENT_SIM == "highway":
                h = vcfg["highway"]

                if vcfg["class"] == "controllable":
                    vehicles[name] = HighwayControllableVehicle(
                        v_index=h["v_index"],
                        vehicle_id=name,
                        vehicle_smt_var=smt_var
                    )
                else:
                    vehicles[name] = HighwayVehicle(
                        v_index=h["v_index"],
                        vehicle_id=name,
                    )

        # Environment erstellen
        if CURRENT_SIM == "carla":
            carla_cfg = cfg["carla"]

            # TM Einstellungen
            tm_settings = {
                "tm_port": carla_cfg.get("tm_port", 8000),
                "sync_mode": carla_cfg.get("sync_mode", True),
                "g_dist_to_leading_vehicle": carla_cfg.get("g_dist_to_leading_vehicle", 2.0),
                "g_perc_speed_diff": carla_cfg.get("g_perc_speed_diff", 0.0),
            }

            # Wetter
            weather_settings = carla_cfg.get("weather", None)

            env = gym.make(
                "CarlaEnv-v0",
                host=CARLA_HOST,
                port=CARLA_PORT,
                map_name=carla_cfg["map_name"],
                action_mode=ACTION_SPACE,
                controllable_vehicles=[v for v in vehicles.values() if isinstance(v, CarlaControllableVehicle)],
                vut_vehicle=vehicles.get("vut"),
                tm_settings=tm_settings,
                weather_settings=weather_settings
            )

        elif CURRENT_SIM == "sumo":
            sumo_cfg = cfg["sumo"]

            env = gym.make(
                "SumoEnv-v0",
                sumo_config_file=str((scenario_dir / sumo_cfg["config_path"]).resolve()),
                action_mode=ACTION_SPACE,
                controllable_vehicles=[v for v in vehicles.values() if isinstance(v, SumoControllableVehicle)],
                vut_vehicle=vehicles.get("vut")
            )

        elif CURRENT_SIM == "highway":
            highway_cfg = cfg["highway"]

            controlled_vehicles_count = sum(1 for v in vehicles.values() if isinstance(v, HighwayControllableVehicle))

            env = gym.make(
                id=highway_cfg.get("id", "highway-v0"),
                render_mode=highway_cfg.get("render_mode", "rgb_array"),
                config={
                    "controlled_vehicles": controlled_vehicles_count,
                    "vehicles_count": len(vehicles) - controlled_vehicles_count,
                    "simulation_frequency": HIGHWAY_SIMULATION_FREQUENCY,
                    "policy_frequency": HIGHWAY_POLICY_FREQUENCY
                }
            )
            env.unwrapped.config.update({
                "action": {
                    "type": "MultiAgentAction",
                    "action_config": {
                        "type": "DiscreteMetaAction",
                        "lateral": True,
                        "longitudinal": True
                    }
                },
                "observation": {
                    "type": "MultiAgentObservation",
                    "observation_config": {"type": "Kinematics"}
                },
                "lanes_count": highway_cfg.get("lanes_count", 3),
                "screen_height": highway_cfg.get("screen_height", 150),
                "screen_width": highway_cfg.get("screen_width", 1200)
            })

        else:
            print("Unbekannter Simulator!")
            return None

        # Reset Environment
        if env_seed is not None:
            env.reset(seed=env_seed)
        else:
            env.reset()

        # Highway-Env Spezifische Einstellungen für die Fahrzeuge (muss nach Reset kommen)
        if CURRENT_SIM == "highway":
            for name, vehicle in vehicles.items():
                vehicle.set_env(env)

                if isinstance(vehicle, HighwayControllableVehicle):
                    default_target_speeds = {"start": -5, "stop": 45, "num": 50}

                    # Hole die target_speeds aus dem JSON, falls vorhanden, sonst default
                    ts_cfg = cfg["vehicles"][name].get("target_speeds", default_target_speeds)

                    start = ts_cfg.get("start", -5)
                    stop = ts_cfg.get("stop", 45)
                    num = ts_cfg.get("num", 50)

                    vehicle.env_vehicle.target_speeds = np.linspace(start, stop, num)

        return env, vehicles
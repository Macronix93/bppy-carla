from bppy import *
from z3 import *

from src.config import ACTION_SPACE
from src.scenario_loader import ScenarioLoader
from src.vehicle.sumo_vehicle import *
from src.action_enum import *

# ------------------------------------------------------------------
# LOAD SCENARIO
# ------------------------------------------------------------------

loader = ScenarioLoader()
env, vehicles = loader.load_scenario("emergency_brake")

v1 = vehicles["veh_manual_1"]
v2 = vehicles["veh_manual_2"]
vut = vehicles["vut"]

step_count = 0

# ------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------

def emergency_brake(vehicle):
    if vehicle.speed() > 0.0:
        yield sync(request=vehicle.SLOWER())
    else:
        yield sync(request=vehicle.IDLE())

def drive(vehicle, target_speed=10.0, speed_eps=0.05):
    if vehicle.speed() < target_speed - speed_eps:
        yield sync(request=vehicle.FASTER())
    else:
        yield sync(request=vehicle.IDLE())

# ------------------------------------------------------------------
# BPpy THREADS
# ------------------------------------------------------------------

@thread
def drive_behaviour_v2():
    if ACTION_SPACE == "micro":
        while True:
            yield from drive(v2, 12.0)

    elif ACTION_SPACE == "macro":
        v2.set_macro_params(target_speed=12.0)

        while True:
            yield sync(request=v2.DRIVE())

@thread
def emergency_brake_v1():
    if ACTION_SPACE == "micro":
        # Normal fahren bis 30m hinter vut
        while v1.is_behind(vut, 30.0):
            yield from drive(v1, 13.0)

        # Stehenbleiben, bis v2 vorbei ist
        while not v2.is_ahead_of(v1, 10.0):
            yield from emergency_brake(v1)

        # Erst danach wieder beschleunigen
        while v1.speed() < 3.0:
            yield from drive(v1, 4.0)

        # Lane Change nach links
        while v1.lane_index() != v2.lane_index():
            yield sync(request=v1.LANE_LEFT())

        # Normal weiterfahren
        while True:
            yield from drive(v1, 12.0)

    elif ACTION_SPACE == "macro":
        v1.set_macro_params(target_speed=13.0, target_lane="left")

        while v1.is_behind(vut, 30.0):
            yield sync(request=v1.DRIVE())

        while v1.is_behind(vut, 10.0) and not v2.is_ahead_of(v1, 10.0):
            yield sync(request=v1.EMERGENCY_BRAKE())

        v1.set_macro_params(target_speed=4.0)

        while v1.speed() < 3.0:
            yield sync(request=v1.DRIVE())

        while v1.lane_index() != v2.lane_index():
            yield sync(request=v1.CHANGE_LANE())

        v1.set_macro_params(target_speed=12.0)

        while True:
            yield sync(request=v1.DRIVE())


@thread
def env_bthread():
    global step_count

    current_action_map = action_map if ACTION_SPACE == "micro" else action_map_macro

    while True:
        e = yield sync(waitFor=true)

        actions = []

        for vehicle in vehicles.values():
            if isinstance(vehicle, BaseControllableVehicle):
                var = vehicle.vehicle_smt_var
                action_vehicle = e.eval(var)

                # fallback auf IDLE/DRIVE, falls Action nicht vorhanden
                actions.append(current_action_map.get(action_vehicle, 1))

        actions_tuple = tuple(actions)

        obs, reward, terminated, truncated, _ = env.step(actions_tuple)
        print(f"OBSERVATION in step {step_count}: {obs}")
        step_count += 1

# ------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------

if __name__ == "__main__":
    try:
        b_program = BProgram(
            bthreads=[
                env_bthread(),
                drive_behaviour_v2(),
                emergency_brake_v1()
            ],
            event_selection_strategy=SMTEventSelectionStrategy(),
            listener=PrintBProgramRunnerListener(),
        )
        b_program.run()
    except Exception as e:
        print("Fehler aufgetreten: ", e)
    finally:
        env.close()
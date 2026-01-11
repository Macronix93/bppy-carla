from bppy import *
from z3 import *

from src.action_enum import action_map, action_map_macro
from src.scenario_loader import ScenarioLoader
from src.vehicle.carla_vehicle import *
from src.config import ACTION_SPACE, GLOBAL_FIXED_DELTA_SECONDS

# ------------------------------------------------------------------
# LOAD SCENARIO
# ------------------------------------------------------------------

loader = ScenarioLoader()
env, vehicles = loader.load_scenario("overtake")

v1 = vehicles["veh_manual_1"]
vut = vehicles["vut"]

# Szenario-Variablen
MIN_DISTANCE = 15.0
MAX_DISTANCE = 20.0
IN_FRONT_DISTANCE = 5.0
OVERTAKE_SPEED = 25.0
DRIVE_SPEED = 20.0

step_count = 0

# ------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------

def seconds(steps):
    return steps * GLOBAL_FIXED_DELTA_SECONDS

def follow_until_close_enough(vehicle, target, min_distance=5.0, max_distance=20.0):
    while (
        vehicle.lane_index() == target.lane_index()  # auf derselben Lane
        and (
                not vehicle.is_behind(target, min_distance)  # noch zu nah
                or vehicle.is_behind(target, max_distance)   # noch zu weit
        )
    ):
        yield from drive(vehicle, DRIVE_SPEED)

def drive(vehicle, target_speed=10.0, speed_eps=0.05):
    if vehicle.speed() < target_speed - speed_eps:
        yield sync(request=vehicle.FASTER())
    else:
        yield sync(request=vehicle.IDLE())

def overtake(vehicle, target, in_front_distance=15.0, overtake_speed=20.0):
    while vehicle.lane_index() == target.lane_index():
        yield sync(request=vehicle.LANE_LEFT())

    # Solange nicht X Meter vor target
    while (not vehicle.is_ahead_of(target, in_front_distance)
           and vehicle.lane_index() != target.lane_index()):
        yield from drive(vehicle, overtake_speed)

    while vehicle.lane_index() != target.lane_index():
        yield sync(request=vehicle.LANE_RIGHT())

def await_condition(
    condition_function, deadline_seconds=float("inf"), local_reward=0.0
) -> Bool:
    global step_count
    step_count_t0 = step_count
    while seconds(step_count - step_count_t0) <= deadline_seconds:
        if condition_function():
            return true
        yield sync(waitFor=true, localReward=local_reward)
        print(
            f" +++  waited {seconds(step_count-step_count_t0)} seconds for condition."
        )
    return false

# ------------------------------------------------------------------
# BPpy THREADS
# ------------------------------------------------------------------

@thread
def v1_overtaking_vut():
    if ACTION_SPACE == "micro":
        while True:
            yield from follow_until_close_enough(v1, vut, min_distance=MIN_DISTANCE, max_distance=MAX_DISTANCE)
            yield from overtake(v1, vut, in_front_distance=IN_FRONT_DISTANCE, overtake_speed=OVERTAKE_SPEED)

    elif ACTION_SPACE == "macro":
        v1.set_macro_params(target_speed=DRIVE_SPEED, target=vut, in_front_distance=IN_FRONT_DISTANCE,
                            overtake_speed=OVERTAKE_SPEED)

        while True:
            while (
                    v1.lane_index() == vut.lane_index()  # auf derselben Lane
                    and (
                            not v1.is_behind(vut, MIN_DISTANCE)  # noch zu nah
                            or v1.is_behind(vut, MAX_DISTANCE)  # noch zu weit
                    )
            ):
                yield sync(request=v1.DRIVE())
            yield sync(request=v1.OVERTAKE())

@thread
def abstract_scenario_v1_overtakes_vut():
    # cond 1.
    satisfied = yield from await_condition(
        lambda: v1.lane_index() == vut.lane_index() and
                v1.is_behind(vut, MIN_DISTANCE) and
                not v1.is_behind(vut, MAX_DISTANCE),
        deadline_seconds=12,
        local_reward=5.0
    )
    if not satisfied:
        print("### UNSAT: v1 konnte sich nicht hinter dem VUT positionieren")
        return
    else:
        print("### SAT: v1 hinter VUT")

    # cond 2.
    satisfied = yield from await_condition(
        lambda: v1.lane_index() != vut.lane_index() and v1.is_ahead_of(vut),
        deadline_seconds=15,
        local_reward=10.0
    )
    if not satisfied:
        print("### UNSAT: v1 konnte nicht überholen")
        return
    else:
        print("### SAT: v1 hat VUT überholt")

    # cond 3.
    satisfied = yield from await_condition(
        lambda: v1.lane_index() == vut.lane_index(),
        deadline_seconds=5,
        local_reward=5.0
    )
    if not satisfied:
        print("### UNSAT: v1 konnte sich nicht wieder einordnen")
        return
    else:
        print("### SAT: v1 wieder korrekt eingeordnet")

    print("### ABSTRACT SCENARIO SAT: v1 successfully overtook VUT")


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

        env.render()

# ------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------

if __name__ == "__main__":
    try:
        b_program = BProgram(
            bthreads=[
                env_bthread(),
                v1_overtaking_vut(),
                abstract_scenario_v1_overtakes_vut()
            ],
            event_selection_strategy=SMTEventSelectionStrategy(),
            listener=PrintBProgramRunnerListener(),
        )
        b_program.run()
    except Exception as e:
        print("Fehler aufgetreten: ", e)
    finally:
        env.close()
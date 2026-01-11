from bppy import *
from z3 import *

from src.action_enum import action_map, action_map_macro
from src.scenario_loader import ScenarioLoader
from src.vehicle.carla_vehicle import *
from src.config import GLOBAL_FIXED_DELTA_SECONDS, ACTION_SPACE

# ------------------------------------------------------------------
# LOAD SCENARIO
# ------------------------------------------------------------------

loader = ScenarioLoader()
env, vehicles = loader.load_scenario("cut_in")

v1 = vehicles["veh_manual_1"]
v2 = vehicles["veh_manual_2"]
vut = vehicles["vut"]

IN_FRONT_GAP = 15.0
BEHIND_GAP = 20.0
INITIAL_GAP = 15.0
SAFE_GAP = 20.0
LANE_MATCH_TIMEOUT = 8
CUTIN_COMPLETE_TIMEOUT = 12

step_count = 0

# ------------------------------------------------------------------
# HELPER FUNCTIONS
# ------------------------------------------------------------------

def wait_seconds(seconds):
    step_count_t0 = step_count
    target_step_count = int(seconds / GLOBAL_FIXED_DELTA_SECONDS) + step_count
    while step_count < target_step_count:
        print(f"waited {(step_count - step_count_t0) * GLOBAL_FIXED_DELTA_SECONDS} seconds.")
        yield sync(request=true)

def seconds(steps):
    return steps * GLOBAL_FIXED_DELTA_SECONDS

def fall_behind(
    behind_vehicle: BaseControllableVehicle,
    in_front_vehicle: BaseVehicle,
    min_distance=25.0,
    max_duration=float("inf"),
):
    global step_count
    step_count_t0 = step_count
    while not behind_vehicle.is_behind(in_front_vehicle, min_distance):
        # behind_vehicle must slow down, but only until it is 2.0 slower than in_front_vehicle
        if behind_vehicle.speed() + 2.0 > in_front_vehicle.speed():
            yield sync(request=behind_vehicle.SLOWER())
        elif behind_vehicle.speed() - 2.0 < in_front_vehicle.speed():
            yield sync(request=behind_vehicle.FASTER())
        else:
            yield sync(request=behind_vehicle.IDLE())
        if seconds(step_count - step_count_t0) >= max_duration:
            print("TIMED INTERRUPT")
            break

def change_to_same_lane(
    vehicle_to_change_lane: BaseControllableVehicle, other_vehicle: BaseVehicle
):
    while vehicle_to_change_lane.lane_index() != other_vehicle.lane_index():
        if vehicle_to_change_lane.lane_index() < other_vehicle.lane_index():
            yield sync(request=vehicle_to_change_lane.LANE_LEFT())
        else:
            yield sync(request=vehicle_to_change_lane.LANE_RIGHT())

def close_distance(
    behind_vehicle: BaseControllableVehicle,
    in_front_vehicle: BaseVehicle,
    max_distance=25.0,
    max_duration=float("inf"),
):
    global step_count
    step_count_t0 = step_count
    while behind_vehicle.is_behind(in_front_vehicle, max_distance):
        # behind_vehicle must speed up down, but only until it is 2.0 faster than in_front_vehicle
        if behind_vehicle.speed() - 2.0 < in_front_vehicle.speed():
            yield sync(request=behind_vehicle.FASTER())
        elif behind_vehicle.speed() + 2.0 > in_front_vehicle.speed():
            yield sync(request=behind_vehicle.SLOWER())
        else:
            yield sync(request=behind_vehicle.IDLE())
        if seconds(step_count - step_count_t0) >= max_duration:
            print("TIMED INTERRUPT")
            break

def equalize_speeds(
    controllable_vehicle: BaseControllableVehicle, other_vehicle: BaseVehicle
):
    while (
        abs(controllable_vehicle.speed() - other_vehicle.speed()) <= 0.1
    ):
        if controllable_vehicle.speed() > other_vehicle.speed():
            yield sync(request=controllable_vehicle.SLOWER())
        else:
            yield sync(request=controllable_vehicle.FASTER())

def get_behind(behind_vehicle: BaseControllableVehicle, in_front_vehicle: BaseVehicle):
    yield from fall_behind(behind_vehicle, in_front_vehicle)
    yield from change_to_same_lane(behind_vehicle, in_front_vehicle)
    yield from close_distance(behind_vehicle, in_front_vehicle)
    yield from equalize_speeds(behind_vehicle, in_front_vehicle)

def stay_behind(behind_vehicle: BaseControllableVehicle, in_front_vehicle: BaseVehicle):
    while True:
        yield from fall_behind(behind_vehicle, in_front_vehicle, 20.0)
        yield from change_to_same_lane(behind_vehicle, in_front_vehicle)
        yield from close_distance(behind_vehicle, in_front_vehicle, 20.0)
        yield from equalize_speeds(behind_vehicle, in_front_vehicle)

def drive(vehicle, target_speed=10.0, speed_eps=0.05):
    if vehicle.speed() < target_speed - speed_eps:
        yield sync(request=vehicle.FASTER())
    else:
        yield sync(request=vehicle.IDLE())

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
def follow_behind(
    behind_vehicle: BaseControllableVehicle,
    in_front_vehicle: BaseVehicle,
    delay_seconds: float = 0.0,
):
    # " serial: "
    yield from get_behind(behind_vehicle, in_front_vehicle)
    yield from stay_behind(behind_vehicle, in_front_vehicle)

@thread
def v2_driving_behaviour():
    if ACTION_SPACE == "micro":
        # Bis V1 neben bzw. vor V2 ist, kleiner Abstand zum VUT halten
        while not v1.is_ahead_of(v2):
            yield from fall_behind(v2, vut, INITIAL_GAP)
            yield from close_distance(v2, vut, INITIAL_GAP)
            yield from equalize_speeds(v2, vut)

        # Sobald V1 neben bzw vor V2, Distanz aufbauen zu V1
        while True:
            yield from fall_behind(v2, v1, 20.0)
            yield from close_distance(v2, v1, 20.0)

    elif ACTION_SPACE == "macro":
        v2.set_macro_params(in_front_vehicle=vut, min_distance=INITIAL_GAP, max_distance=INITIAL_GAP)
        while not v1.is_ahead_of(v2):
            yield sync(request=v2.APPROACH_AND_MATCH())

        v2.set_macro_params(in_front_vehicle=v1, min_distance=20.0, max_distance=20.0)
        while True:
            yield sync(request=v2.FOLLOW_AT_SPEED())

@thread
def cut_in_v1_between_vut_and_v2():
    if ACTION_SPACE == "micro":
        # v1 muss erst bis zu vut aufholen
        while v1.is_behind(vut, INITIAL_GAP):
            yield from drive(v1, 25.0)

        # Wenn aufgeholt wurde muss geschaut werden ob die Lücke passt
        while not v1.is_behind(vut, BEHIND_GAP) and not v1.is_ahead_of(v2, IN_FRONT_GAP):
            yield from fall_behind(v1, vut, SAFE_GAP)
            yield from close_distance(v1, vut, SAFE_GAP)
            yield from equalize_speeds(v1, vut)

        # Cut-In ausführen
        while v1.lane_index() != vut.lane_index():
            yield sync(request=v1.LANE_RIGHT())

        # Danach normal folgen
        while True:
            yield from follow_behind(v1, vut)

    elif ACTION_SPACE == "macro":
        # Aufholen
        v1.set_macro_params(target_speed=25.0, in_front_vehicle=vut, behind_vehicle=v2,
                            min_distance=SAFE_GAP, max_distance=SAFE_GAP, safe_gap=SAFE_GAP)

        while v1.is_behind(vut, INITIAL_GAP):
            yield sync(request=v1.DRIVE())

        # Warten bis Lücke passt
        while not v1.is_behind(vut, BEHIND_GAP) and not v1.is_ahead_of(v2, IN_FRONT_GAP):
            yield sync(request=v1.APPROACH_AND_MATCH())

        # CUT-IN ausführen
        while v1.lane_index() != vut.lane_index():
            yield sync(request=v1.CUT_IN())

        # Danach normal folgen
        v1.set_macro_params(min_distance=20.0, max_distance=20.0)

        while True:
            yield sync(request=v1.FOLLOW_AT_SPEED())

@thread
def abstract_scenario_cut_in():
    print("### ABSTRACT CUT-IN STARTED")

    # cond 1.
    # v1 befindet sich hinter dem VUT
    satisfied = yield from await_condition(
        lambda: v1.is_behind(vut, BEHIND_GAP) and v1.is_ahead_of(v2, IN_FRONT_GAP),
        deadline_seconds=20,
        local_reward=5.0
    )
    if not satisfied:
        print("### UNSAT: v1 war nicht hinter dem VUT")
        return
    else:
        print("### COND 1 SAT: v1 hinter VUT")

    # cond 2.
    # v1 wechselt in die Spur des VUT und V2
    satisfied = yield from await_condition(
        lambda: v1.lane_index() == vut.lane_index(),
        deadline_seconds=LANE_MATCH_TIMEOUT,
        local_reward=10.0
    )
    if not satisfied:
        print("### UNSAT: v1 hat Spur nicht gewechselt")
        return
    else:
        print("### COND 2 SAT: gleiche Spur wie VUT")

    # cond 3.
    # v1 ist nun zwischen dem VUT und V2
    satisfied = yield from await_condition(
        lambda: v1.is_behind(vut, BEHIND_GAP) and v1.is_ahead_of(v2, IN_FRONT_GAP),
        deadline_seconds=CUTIN_COMPLETE_TIMEOUT,
        local_reward=20.0
    )
    if not satisfied:
        print("### UNSAT: v1 ist nicht vor dem VUT angekommen")
        return
    else:
        print("### COND 3 SAT: v1 vor VUT — CUT-IN COMPLETE")
    print("### SAT: CUT-IN ABSTRACT SCENARIO")

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
                v2_driving_behaviour(),
                cut_in_v1_between_vut_and_v2(),
                abstract_scenario_cut_in()
            ],
            event_selection_strategy=SMTEventSelectionStrategy(),
            listener=PrintBProgramRunnerListener(),
        )
        b_program.run()
    except Exception as e:
        print("Fehler aufgetreten: ", e)
    finally:
        env.close()
from bppy import *
from z3 import *

from src.config import GLOBAL_FIXED_DELTA_SECONDS, ACTION_SPACE
from src.scenario_loader import ScenarioLoader
from src.vehicle.sumo_vehicle import *
from src.action_enum import *

# ------------------------------------------------------------------
# LOAD SCENARIO
# ------------------------------------------------------------------

loader = ScenarioLoader()
env, vehicles = loader.load_scenario("follow_behind")

v1 = vehicles["veh_manual_1"]
v2 = vehicles["veh_manual_2"]
vut = vehicles["vut"]

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
        # behind_vehicle must speed up/down, but only until it is 2.0 faster than in_front_vehicle
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
    while abs(controllable_vehicle.speed() - other_vehicle.speed()) <= 0.1:
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

def parallel(*bthreads):
    for bt in bthreads:
        b_program.add_bthread(bt)
    yield sync(waitFor=true)  # needs to be here, otherwise there might be problem w

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
    if ACTION_SPACE == "micro":
        yield from get_behind(behind_vehicle, in_front_vehicle)
        yield from stay_behind(behind_vehicle, in_front_vehicle)

    elif ACTION_SPACE == "macro":
        behind_vehicle.set_macro_params(target=in_front_vehicle, min_distance=20.0, max_distance=20.0)

        while True:
            yield sync(request=behind_vehicle.FOLLOW_VEHICLE())

@thread
def two_vehicles_follow_vut():
    # v1 follows vut, v2 follows v1
    yield from parallel(follow_behind(v1, vut), follow_behind(v2, v1))

# abstract scenario:
#   - some sequence or control flow of conditions or events that we wait for / monitor
#   - or some composition of other scenarios.
@thread
def abstract_scenario_two_vehicles_follow_vut():
    def condition():
        return v1.is_behind(vut) and v2.is_behind(v1)  # TODO: Same lane?

    satisfied = yield from await_condition(condition, 10)
    if satisfied:
        print("################ SAT abstract_scenario_two_vehicles_follow_vut")
    else:
        print("################ UNSAT abstract_scenario_two_vehicles_follow_vut")

@thread
def abstract_scenario_2():
    # cond 1.
    satisfied = yield from await_condition(
        lambda: v1.is_behind(vut), local_reward=10.0
    )
    if not satisfied:
        print("################ UNSAT")
        return
    else:
        print("################ COND 1 SAT")

    # cond 2.
    satisfied = yield from await_condition(
        lambda: v1.lane_index() == vut.lane_index(), local_reward=10.0
    )
    if not satisfied:
        print("################ UNSAT")  # -> local_reward = -100.0 ???
        return
    else:
        print("################ COND 2 SAT")

    # cond 3.
    satisfied = yield from await_condition(
        lambda: v2.is_behind(v1), local_reward=10.0
    )
    if not satisfied:
        print("################ UNSAT")
        return
    else:
        print("################ COND 3 SAT")

    # cond 4.
    satisfied = yield from await_condition(
        lambda: v2.lane_index() == v1.lane_index(), local_reward=10.0
    )
    if not satisfied:
        print("################ UNSAT")
        return
    else:
        print("################ COND 4 SAT")
    print("################ SAT")

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
    # Creating a BProgram with the defined b-threads, SMTEventSelectionStrategy,
    # and a listener to print the selected events
    try:
        b_program = BProgram(
            bthreads=[
                env_bthread(),
                two_vehicles_follow_vut(), # concrete scenarios / manually crafted implementation -> to be learned by RL
                abstract_scenario_2(),
                abstract_scenario_two_vehicles_follow_vut(),
            ],  # abstract scenarios / declarative / spec / target / rewards
            event_selection_strategy=SMTEventSelectionStrategy(),
            listener=PrintBProgramRunnerListener(),
        )
        b_program.run()
    except Exception as e:
        print("Fehler aufgetreten: ", e)
    finally:
        env.close()
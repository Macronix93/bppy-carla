from gymnasium.envs.registration import register

register(id="SumoEnv-v0", entry_point="src.env.sumo_env:SumoEnv")
register(id="CarlaEnv-v0", entry_point="src.env.carla_env:CarlaEnv")
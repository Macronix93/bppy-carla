import time
from typing import Optional, List

import carla
import numpy as np

from src.agents.navigation.local_planner import LocalPlanner
from src.env.base_env import BaseEnv
from src.carla_server import CarlaServer
from src.config import GLOBAL_FIXED_DELTA_SECONDS, CARLA_QUALITY, CARLA_DEBUG, CARLA_SPECTATE, CARLA_TIMEOUT, CARLA_GUI, \
    CARLA_SPECTATE_MODE

from src.vehicle.carla_vehicle import CarlaVehicle, CarlaControllableVehicle

class CarlaEnv(BaseEnv):

    def __init__(self, host: str, port: int, map_name: str, controllable_vehicles: List[CarlaControllableVehicle],
                 vut_vehicle: CarlaVehicle, action_mode="micro", tm_settings: Optional[dict] = None,
                 weather_settings: Optional[dict] = None):
        super().__init__(controllable_vehicles, vut_vehicle, action_mode)

        # CARLA Variablen
        self.host = host
        self.port = port
        self.map_name = map_name
        self.carla_server = CarlaServer(CARLA_QUALITY)
        self.client = None
        self.world = None
        self.tm_settings = tm_settings
        self.weather_settings = weather_settings

        # Kollisionsstatus für jedes Fahrzeug speichern
        self.collision_status = {v.vehicle_id: False for v in self.all_vehicles}
        self.collision_sensors = {}  # Für die Speicherung der Sensor-Aktoren

    # ------------------------------------------------------------------
    # RESET, STEP
    # ------------------------------------------------------------------

    def reset(self, seed: Optional[int] = None, **kwargs):
        if self.episode != 0:
            self.close()

        self.episode += 1
        self.step_count = 0

        # Kollisionsstatus für alle Fahrzeuge zurücksetzen
        self.collision_status = {v.vehicle_id: False for v in self.all_vehicles}
        self.collision_sensors = {}  # Auch Sensoren zurücksetzen

        self._start_simulation(carla_gui=CARLA_GUI)
        self.build_vehicles()

        # Erster Simulationsschritt im Sync-Modus
        self.world.tick()

        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        start = time.time()

        # Kollisionsvariable resetten
        for v in self.all_vehicles:
            self.collision_status[v.vehicle_id] = False

        self._apply_action(action)
        self.world.tick()
        self.step_count += 1

        # Einem Fahrzeug mit der World Cam folgen
        if CARLA_SPECTATE is not None:
            self.controllable_vehicles[CARLA_SPECTATE].spectate(mode=CARLA_SPECTATE_MODE)

        obs = self._get_obs()
        reward = self._get_reward()
        terminated = self.step_count >= self.max_steps or any(self.collision_status.values())
        truncated = False

        # Sonst ist die Simulation zu schnell und nicht vergleichbar mit anderen Simulatoren
        elapsed = time.time() - start
        if elapsed < GLOBAL_FIXED_DELTA_SECONDS:
            time.sleep(GLOBAL_FIXED_DELTA_SECONDS - elapsed)

        return obs, reward, terminated, truncated, {}

    # ------------------------------------------------------------------
    # SIMULATION SETUP
    # ------------------------------------------------------------------

    def _start_simulation(self, carla_gui: bool = False):
        print(f"Verbinde zu CARLA: {self.host}:{self.port} ...")

        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(CARLA_TIMEOUT)

        try:
            # Falls der Server erreichbar ist, wird hier NICHT geworfen
            _world = self.client.get_world()
            server_running = True
        except Exception:
            server_running = False

        if not server_running:
            self.carla_server.start_carla(offscreen_rendering=not carla_gui, sleep_time=CARLA_TIMEOUT)

            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(CARLA_TIMEOUT)
            except Exception as e:
                raise RuntimeError(f"CARLA konnte nicht gestartet werden: {e}")

            # Map laden
            print(f"Lade Map '{self.map_name}' ...")
            self.world = self.client.load_world(self.map_name)
        else:
            current_map = self.client.get_world().get_map().name.split("/")[-1] # get.map().name liefert "Carla/Maps/Mapname"

            if current_map != self.map_name:
                print(f"-> Lade neue Map...")

                self.world = self.client.load_world(self.map_name)

            else:
                print("-> Map stimmt überein. Nutze bestehende Welt.")
                self.world = self.client.get_world()

        # Traffic Manager konfigurieren
        self.tm = self.client.get_trafficmanager(self.tm_settings["tm_port"])
        self.tm_port = self.tm.get_port()
        self.tm.set_synchronous_mode(self.tm_settings["sync_mode"])
        self.tm.set_global_distance_to_leading_vehicle(self.tm_settings["g_dist_to_leading_vehicle"])
        self.tm.global_percentage_speed_difference(self.tm_settings["g_perc_speed_diff"])

        # Welt konfigurieren
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = GLOBAL_FIXED_DELTA_SECONDS
        settings.no_rendering_mode = not carla_gui
        self.world.apply_settings(settings)

        # Wetter setzen
        ws = self.weather_settings
        if ws is not None:
            if "preset" in ws:
                try:
                    weather = getattr(carla.WeatherParameters, ws["preset"])
                except AttributeError:
                    raise ValueError(f"Unbekanntes CARLA Wetter Preset: {ws["preset"]}")
            else:
                weather = carla.WeatherParameters(**ws)

            # Benutzerdefinierte Werte überschreiben
            for k, v in ws.items():
                if k != "preset":
                    setattr(weather, k, v)

            self.world.set_weather(weather)

        # Spawnpunkte laden
        self.spawn_points = self.world.get_map().get_spawn_points()

        # DEBUG: Zeichnet Marker an allen Spawnpunkten
        if CARLA_DEBUG:
            for i, sp in enumerate(self.spawn_points):
                loc = sp.location
                # Index als Text darüber
                self.world.debug.draw_string(loc + carla.Location(z=0.5), str(i), draw_shadow=True,
                                             color=carla.Color(255, 0, 0), life_time=200.0, persistent_lines=True)

        print(f"CARLA Simulation gestartet auf Karte '{self.map_name}'.")

    # ------------------------------------------------------------------
    # VEHICLES
    # ------------------------------------------------------------------

    def build_vehicles(self):
        blueprint_library = self.world.get_blueprint_library()

        for vehicle in self.all_vehicles:

            # Blueprint laden
            try:
                # ID benutzen, um Blueprint-Objekt zu finden
                vehicle.carla_blueprint = blueprint_library.find(vehicle.carla_blueprint_id)
            except Exception as e:
                print(
                    f"Fehler: Konnte Blueprint '{vehicle.carla_blueprint_id}' für {vehicle.vehicle_id} nicht finden! {e}")
                continue

            # Spawnpunkt zuweisen
            if vehicle.spawn_index is None or vehicle.spawn_index >= len(
                    self.spawn_points) or vehicle.spawn_index < 0:
                print(f"Warnung: Ungültiger Spawnindex für {vehicle.vehicle_id}. Überspringe Spawn.")
                continue

            spawn_transform = self.spawn_points[vehicle.spawn_index]
            vehicle.spawn_point = spawn_transform

            # Actor spawnen und Einstellungen setzen
            if vehicle.carla_blueprint.has_attribute("color"):
                r, g, b = vehicle.vehicle_color
                vehicle.carla_blueprint.set_attribute("color", f"{r},{g},{b}")

            actor = self.world.try_spawn_actor(vehicle.carla_blueprint, vehicle.spawn_point)

            if actor:
                is_controllable = isinstance(vehicle, CarlaControllableVehicle)
                actor.set_autopilot(not is_controllable, self.tm_port)

                # Actor-Spawning abschließen, damit Initialgeschwindigkeit funktioniert
                self.world.tick()

                # Initialgeschwindigkeit setzen (in m/s)
                initial_speed = vehicle.initial_speed
                initial_speed_kmh = initial_speed * 3.6

                # Geschwindigkeit entlang der Fahrzeug-Richtung
                forward = actor.get_transform().get_forward_vector()
                actor.set_target_velocity(
                    carla.Vector3D(forward.x * initial_speed,
                                   forward.y * initial_speed,
                                   forward.z * initial_speed)
                )

                if vehicle.vehicle_id == "vut":
                    vehicle_tm_settings = {
                        "auto_lane_change": True,
                        "ignore_lights_percentage": 100,
                        "ignore_signs_percentage": 100
                    }

                    # Falls JSON-Werte vorhanden sind -> nur vorhandene Keys überschreiben
                    if vehicle.tm_settings is not None:
                        for key, val in vehicle.lp_opts.items():
                            if key in vehicle_tm_settings:
                                vehicle_tm_settings[key] = val  # Default überschreiben

                    self.tm.auto_lane_change(actor, vehicle_tm_settings["auto_lane_change"])
                    self.tm.ignore_lights_percentage(actor, vehicle_tm_settings["ignore_lights_percentage"])
                    self.tm.ignore_signs_percentage(actor, vehicle_tm_settings["ignore_signs_percentage"])
                    self.tm.set_desired_speed(actor, initial_speed_kmh)

                vehicle.carla_actor = actor

                # Sensor spawnen und an Fahrzeug anbringen
                collision_sensor = self.world.try_spawn_actor(
                    blueprint_library.find('sensor.other.collision'), carla.Transform(), attach_to=actor)

                if collision_sensor:
                    # Callback registrieren
                    collision_sensor.listen(self._on_collision)

                    # Sensor-Objekt speichern
                    self.collision_sensors[vehicle.vehicle_id] = collision_sensor
                else:
                    print(f"Fehler: Konnte Kollisions-Sensor nicht an {vehicle.vehicle_id} spawnen.")

                if is_controllable:
                    # LocalPlanner für das steuerbare Fahrzeug
                    # Standardwerte für Vehicle PID Controller
                    lp_opts = {
                        "lateral_control_dict": {"K_P": 1.2, "K_I": 0.02, "K_D": 0.1, "dt": GLOBAL_FIXED_DELTA_SECONDS},
                        "longitudinal_control_dict": {"K_P": 1.0, "K_I": 0.05, "K_D": 0, "dt": GLOBAL_FIXED_DELTA_SECONDS},
                        "max_throttle": 1.0,
                        "max_brake": 0.5,
                        "max_steering": 0.8,
                    }

                    # Falls Fahrzeug andere Werte für LocalPlanner hat (aus JSON) -> überschreiben
                    if vehicle.lp_opts is not None:
                        for key, val in vehicle.lp_opts.items():
                            if isinstance(val, dict) and key in lp_opts:
                                lp_opts[key].update(val)
                            else:
                                lp_opts[key] = val

                    planner = LocalPlanner(actor, opt_dict=lp_opts, map_inst=self.world.get_map())
                    planner._stop_waypoint_creation = False
                    planner._min_waypoint_queue_length = 80
                    planner.set_speed(initial_speed_kmh)

                    vehicle.planners = planner

                print(
                    f"Erstelle Fahrzeug: {vehicle.vehicle_id} (Controllable: {is_controllable}) bei Index {vehicle.spawn_index} mit Farbe {vehicle.vehicle_color}.")
            else:
                print(f"Warnung: Konnte Fahrzeug {vehicle.vehicle_id} bei Index {vehicle.spawn_index} nicht spawnen.")

    def _on_collision(self, event):
        actor = event.actor
        other_actor = event.other_actor

        vehicle_id = None
        other_vehicle_id = None

        # Finde unser Fahrzeug
        for v in self.all_vehicles:
            if v.carla_actor.id == actor.id:
                vehicle_id = v.vehicle_id
            if v.carla_actor.id == other_actor.id:
                other_vehicle_id = v.vehicle_id

        # Status setzen
        if vehicle_id:
            self.collision_status[vehicle_id] = True
            print(f"Kollision erkannt: {vehicle_id} mit {other_vehicle_id}")

    # ------------------------------------------------------------------
    # LOW LEVEL ACTION
    # ------------------------------------------------------------------

    def _apply_low_level_action(self, veh, action):
        planner = veh.planners
        speed = veh.speed()  # m/s

        if action == 0:  # LANE_LEFT
            veh.request_lane_change("left")
            veh.carla_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.LeftBlinker))
        elif action == 1:  # IDLE
            planner.set_speed(speed * 3.6)
        elif action == 2:  # LANE_RIGHT
            veh.request_lane_change("right")
            veh.carla_actor.set_light_state(carla.VehicleLightState(carla.VehicleLightState.RightBlinker))
        elif action == 3:  # FASTER
            planner.set_speed((speed + 1.0) * 3.6)
        elif action == 4:  # SLOWER
            planner.set_speed(max(0.0, speed - 1.0) * 3.6)

        # Local Planner Step und eventuellen Lane-Change durchführen
        veh.perform_lane_change_step(planner)

    # ------------------------------------------------------------------
    # OBS / TERMINATION
    # ------------------------------------------------------------------

    def render(self):
        pass

    def _get_obs(self) -> np.ndarray:
        obs = []

        for vehicle in self.all_vehicles:
            if vehicle:
                collision_flag = self.collision_status.get(vehicle.vehicle_id, False)  # Holt Kollisionsstatus
                collision_value = 1.0 if collision_flag else 0.0  # Mappe auf 1.0 oder 0.0
                loc = vehicle.get_location()

                obs.append([loc.x, loc.y, vehicle.speed(), vehicle.lane_index(), collision_value])
            else:
                obs.append([0.0, 0.0, 0.0, -1, 0.0])

        return np.array(obs, dtype=np.float32)

    # ------------------------------------------------------------------
    # CLEANUP
    # ------------------------------------------------------------------

    def close(self):
        # Alle Akteure im World zerstören und den Synchronous Mode beenden
        if self.world:
            print("Zerstöre Actors...")

            # Sensoren zerstören
            for sensor in self.collision_sensors.values():
                if sensor is not None:
                    sensor.destroy()
            self.collision_sensors = {}

            # Fahrzeuge zerstören
            for vehicle in self.all_vehicles:
                if vehicle.carla_actor is not None:
                    vehicle.carla_actor.destroy()
                    vehicle.carla_actor = None

            # Zurück zum Asynchronen Modus (wichtig für den Server-Reset)
            settings = self.world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

        # Client-Verbindung beenden (CARLA Server läuft weiter)
        self.client = None
        self.world = None
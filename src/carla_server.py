import os
import signal
import subprocess
import time

import psutil


class CarlaServer:
    def __init__(self, carla_quality):
        self.carla_quality = carla_quality
        self.carla_process = None

    def start_carla(self, offscreen_rendering=False, silent=False, sleep_time=30):
        self.kill_running_carla()  # alte Instanzen beenden

        carla_dir = os.getenv("CARLA_SERVER")

        if carla_dir is None:
            raise RuntimeError("Umgebungsvariable CARLA_SERVER nicht gesetzt!")

        if os.name == 'posix':
            exe_path = os.path.join(carla_dir, "CarlaUE4.sh")
            command = f"bash{exe_path} {"-quality-level=" + self.carla_quality if self.carla_quality else ""} {"-RenderOffScreen" if offscreen_rendering else ""}"
        else:
            exe_path = os.path.join(carla_dir, "CarlaUE4.exe")
            command = f"{exe_path} {"-quality-level=" + self.carla_quality if self.carla_quality else ""} {"-RenderOffScreen" if offscreen_rendering else ""}"

        if not silent:
            print("Starte CARLA Server...")

        self.carla_process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        time.sleep(sleep_time)

        if not silent:
            print("CARLA Server gestartet.")

    def close_carla(self, silent=False):
        if self.carla_process is None:
            return

        if os.name == 'posix':
            try:
                os.killpg(self.carla_process.pid, signal.SIGTERM)
            except Exception:
                pass
        else:
            subprocess.run(['taskkill', '/F', '/T', '/PID', str(self.carla_process.pid)],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        if not silent:
            print("CARLA Server beendet.")

        self.carla_process = None

    def restart_carla(self, offscreen_rendering=False, silent=False, sleep_time=30):
        self.close_carla(silent)
        self.start_carla(offscreen_rendering, silent, sleep_time)

    def is_carla_running(self):
        for proc in psutil.process_iter(['name']):
            try:
                if proc.info['name'] and "CarlaUE4" in proc.info['name']:
                    return proc
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return None

    def kill_running_carla(self):
        if os.name == 'posix':
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    if proc.info['name'] and "CarlaUE4" in proc.info['name']:
                        os.kill(proc.pid, signal.SIGTERM)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        else:
            for proc in psutil.process_iter(['name', 'pid']):
                try:
                    if proc.info['name'] and "CarlaUE4.exe" in proc.info['name']:
                        print(f"Beende laufende CARLA Instanz PID={proc.pid}")
                        subprocess.run(['taskkill', '/F', '/T', '/PID', str(proc.pid)])
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
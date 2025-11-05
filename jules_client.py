import requests
import json
import threading
import time

class JulesClient:
    def __init__(self, base_url, token, on_data, on_error):
        self.base_url = base_url.rstrip('/')
        self.token = token
        self.on_data = on_data
        self.on_error = on_error
        self.data = {}
        self.running = False
        self.thread = None

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self.listen_for_events, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def listen_for_events(self):
        url = f"{self.base_url}/jules/stream?token={self.token}"
        headers = {'Accept': 'text/event-stream'}
        try:
            with requests.get(url, headers=headers, stream=True, timeout=10) as response:
                if response.status_code != 200:
                    self.on_error(f"Connection failed with status code: {response.status_code}")
                    return
                for line in response.iter_lines():
                    if not self.running:
                        break
                    if line.startswith(b'data: '):
                        try:
                            json_data = line.decode('utf-8')[6:]
                            parsed_data = json.loads(json_data)
                            
                            # --- Update data dictionary with all new metrics ---
                            self.data['t'] = parsed_data.get('ts', 0)
                            self.data['cmdPower'] = parsed_data.get('cmd', 0)
                            self.data['velIPS'] = parsed_data.get('vel_ips', 0)
                            self.data['headingDeg'] = parsed_data.get('heading_deg', 0)
                            self.data['batteryV'] = parsed_data.get('battery_v', 0)
                            
                            # Odometry data
                            self.data['x'] = parsed_data.get('x', 0)
                            self.data['y'] = parsed_data.get('y', 0)
                            self.data['heading'] = parsed_data.get('heading', 0) # Radians
                            
                            # Detailed IMU data
                            self.data['pitch'] = parsed_data.get('pitch', 0)
                            self.data['roll'] = parsed_data.get('roll', 0)
                            self.data['yawRate'] = parsed_data.get('yawRate', 0)
                            self.data['pitchRate'] = parsed_data.get('pitchRate', 0)
                            self.data['rollRate'] = parsed_data.get('rollRate', 0)

                            if self.on_data:
                                self.on_data(self.data)
                        except (json.JSONDecodeError, KeyError) as e:
                            self.on_error(f"Error parsing data: {e}")
        except requests.exceptions.RequestException as e:
            if self.running:
                self.on_error(f"Connection error: {e}")
        finally:
            self.running = False
            
    def get_latest_data(self):
        return self.data
# data_stream.py

import requests
import json
import threading
import time
import math
import random
import collections
from metrics_datamodel import MetricsData

class JulesClient:
    """Handles the connection and data streaming from the real robot."""
    def __init__(self, base_url, token, on_data, on_error):
        self.base_url = base_url.rstrip('/')
        self.token = token
        self.on_data = on_data
        self.on_error = on_error
        self.data = MetricsData()
        self.running = False
        self.thread = None

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._listen_for_events, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread: self.thread.join()

    def _listen_for_events(self):
        url = f"{self.base_url}/jules/stream?token={self.token}"
        headers = {'Accept': 'text/event-stream'}
        try:
            # Reverting to iter_lines() with a timeout, just like the original file.
            with requests.get(url, headers=headers, stream=True, timeout=15) as response:
                if response.status_code != 200:
                    self.on_error(f"Connection failed: {response.status_code}")
                    return

                for line in response.iter_lines():
                    if not self.running: 
                        break
                    
                    if line.startswith(b'data: '):
                        try:
                            json_data = line.decode('utf-8')[6:]
                            if not json_data: continue

                            parsed_data = json.loads(json_data)
                            self.data.update_from_dict(parsed_data)
                            if self.on_data:
                                self.on_data(self.data)
                        except (json.JSONDecodeError, KeyError, IndexError) as e:
                            self.on_error(f"Error parsing data: {e} | Line: {line}")

        except requests.exceptions.RequestException as e:
            if self.running:
                self.on_error(f"Connection error: {e}")
        finally:
            self.running = False

    def get_latest_data(self):
        return self.data


class FakeDataGenerator:
    # This class remains unchanged from the previous version
    def __init__(self, on_data, on_error):
        self.on_data = on_data
        self.on_error = on_error
        self.data = MetricsData()
        self.running = False
        self.thread = None
        self.buffer = collections.deque(maxlen=2000)

    def start(self):
        if self.running: return
        self.running = True
        self.thread = threading.Thread(target=self._run_generator_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread: self.thread.join()

    def get_latest_data(self):
        return self.data
        
    def get_buffered_data(self):
        return list(self.buffer)

    def _run_generator_loop(self):
        start_time = time.time()
        while self.running:
            elapsed = time.time() - start_time
            self.data.update_for_fake_generator(elapsed)
            self.buffer.append(self.data.to_dict())
            if self.on_data: self.on_data(self.data)
            time.sleep(0.02)
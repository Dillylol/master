import tkinter as tk
from tkinter import ttk, font
import threading
import time
import math
import random
from jules_client import JulesClient

class JulesTelemetryGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("JULES Telemetry Dashboard")
        self.root.geometry("800x600")

        # Style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.bold_font = font.Font(family="Helvetica", size=10, weight="bold")
        self.mono_font = font.Font(family="Courier", size=10)
        self.style.configure("TLabel", padding=5, font=('Helvetica', 10))
        self.style.configure("Bold.TLabel", padding=5, font=self.bold_font)
        self.style.configure("Header.TLabel", padding=5, font=('Helvetica', 12, 'bold'))

        self.client = None
        self.data = {}
        
        # --- Fake Data Generator ---
        self.use_fake_data = tk.BooleanVar(value=False)
        self.fake_data_thread = None
        self.stop_fake_data_thread = False

        self.create_widgets()
        self.update_data()

    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        
        ttk.Label(conn_frame, text="IP Address:").grid(row=0, column=0, sticky="w")
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.insert(0, "http://192.168.43.1:58080")
        self.ip_entry.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="Token:").grid(row=0, column=2, sticky="w", padx=10)
        self.token_entry = ttk.Entry(conn_frame, width=25)
        self.token_entry.grid(row=0, column=3, padx=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=10)

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=5, padx=10)
        
        # --- Fake Data Checkbox ---
        self.fake_data_check = ttk.Checkbutton(
            conn_frame, text="Use Fake Data", variable=self.use_fake_data, command=self.toggle_fake_data)
        self.fake_data_check.grid(row=0, column=6, padx=20)

        # --- Data Frame ---
        data_frame = ttk.Frame(main_frame)
        data_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5))
        
        # --- Constants Frame ---
        constants_frame = ttk.LabelFrame(main_frame, text="Pedro Pathing Constants", padding="10")
        constants_frame.grid(row=1, column=1, sticky="nsew", padx=(5, 0))
        
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)

        # --- Basic Telemetry ---
        basic_frame = ttk.LabelFrame(data_frame, text="Core Metrics", padding="10")
        basic_frame.pack(fill=tk.X, pady=5)
        self.labels = {}
        core_metrics = ["t", "cmdPower", "velIPS", "headingDeg", "batteryV"]
        for i, metric in enumerate(core_metrics):
            ttk.Label(basic_frame, text=f"{metric}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w")
            self.labels[metric] = ttk.Label(basic_frame, text="0.000", width=15, anchor="w", font=self.mono_font)
            self.labels[metric].grid(row=i, column=1, sticky="w")

        # --- Odometry Telemetry ---
        odom_frame = ttk.LabelFrame(data_frame, text="Odometry", padding="10")
        odom_frame.pack(fill=tk.X, pady=5)
        odom_metrics = ["x", "y", "heading"]
        for i, metric in enumerate(odom_metrics):
            ttk.Label(odom_frame, text=f"{metric}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w")
            self.labels[metric] = ttk.Label(odom_frame, text="0.000", width=15, anchor="w", font=self.mono_font)
            self.labels[metric].grid(row=i, column=1, sticky="w")

        # --- IMU Telemetry ---
        imu_frame = ttk.LabelFrame(data_frame, text="IMU", padding="10")
        imu_frame.pack(fill=tk.X, pady=5)
        imu_metrics = ["pitch", "roll", "yawRate", "pitchRate", "rollRate"]
        for i, metric in enumerate(imu_metrics):
            ttk.Label(imu_frame, text=f"{metric}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w")
            self.labels[metric] = ttk.Label(imu_frame, text="0.000", width=15, anchor="w", font=self.mono_font)
            self.labels[metric].grid(row=i, column=1, sticky="w")

        # --- Constants Widgets ---
        self.entries = {}
        pp_constants = ["MASS", "WHEEL_RADIUS", "GEAR_RATIO", "TRACK_WIDTH", "LATERAL_MULTIPLIER"]
        default_values = ["10.0", "1.88976", "1.0", "15.5", "1.0"] # Example defaults
        for i, const in enumerate(pp_constants):
            ttk.Label(constants_frame, text=f"{const}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w")
            self.entries[const] = ttk.Entry(constants_frame, width=15, font=self.mono_font)
            self.entries[const].insert(0, default_values[i])
            self.entries[const].grid(row=i, column=1, sticky="w", padx=5)
        
        self.send_constants_button = ttk.Button(constants_frame, text="Send to Robot", command=self.send_constants)
        self.send_constants_button.grid(row=len(pp_constants), column=0, columnspan=2, pady=10)

    def toggle_fake_data(self):
        if self.use_fake_data.get():
            if self.client and self.client.running:
                self.toggle_connection() # Disconnect from real robot
            
            self.stop_fake_data_thread = False
            self.fake_data_thread = threading.Thread(target=self.generate_fake_data, daemon=True)
            self.fake_data_thread.start()
            self.status_label.config(text="Status: Using Fake Data", foreground="blue")
        else:
            self.stop_fake_data_thread = True
            if self.fake_data_thread:
                self.fake_data_thread.join()
            self.status_label.config(text="Status: Disconnected", foreground="red")


    def generate_fake_data(self):
        """Generates a fake stream of odometry data for GUI testing."""
        start_time = time.time()
        while not self.stop_fake_data_thread:
            elapsed = time.time() - start_time
            
            # Simulate circular motion
            radius = 24 # inches
            speed = 12  # inches per second
            angular_velocity = speed / radius # rad/s
            
            angle = angular_velocity * elapsed
            
            self.data['t'] = elapsed
            self.data['x'] = radius * math.cos(angle)
            self.data['y'] = radius * math.sin(angle)
            self.data['heading'] = (angle + math.pi / 2) % (2 * math.pi) # Robot heading tangent to circle
            
            # Fake other data
            self.data['velIPS'] = speed + (random.random() - 0.5)
            self.data['headingDeg'] = math.degrees(self.data['heading'])
            self.data['batteryV'] = 12.5 - elapsed * 0.01
            self.data['cmdPower'] = 0.5 + 0.1 * math.sin(elapsed)
            
            self.data['pitch'] = 1.0 + (random.random() - 0.5) * 0.5
            self.data['roll'] = -0.5 + (random.random() - 0.5) * 0.5
            self.data['yawRate'] = math.degrees(angular_velocity) + (random.random() - 0.5)
            self.data['pitchRate'] = (random.random() - 0.5) * 0.2
            self.data['rollRate'] = (random.random() - 0.5) * 0.2

            time.sleep(0.02) # 50Hz update rate

    def toggle_connection(self):
        if self.client and self.client.running:
            self.client.stop()
            self.client = None
            self.connect_button.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")
        else:
            if self.use_fake_data.get():
                self.use_fake_data.set(False)
                self.toggle_fake_data()

            ip = self.ip_entry.get()
            token = self.token_entry.get()
            if not ip or not token:
                self.status_label.config(text="Status: IP and Token required", foreground="red")
                return

            self.client = JulesClient(ip, token, self.on_data_received, self.on_error)
            self.client.start()
            self.connect_button.config(text="Disconnect")
            self.status_label.config(text="Status: Connecting...", foreground="orange")

    def on_data_received(self, data):
        self.data = data
        if self.client and self.client.running:
             self.status_label.config(text="Status: Connected", foreground="green")

    def on_error(self, error_message):
        if self.client and self.client.running:
            self.status_label.config(text=f"Status: Error", foreground="red")
            print(error_message) # Print error to console for debugging

    def send_constants(self):
        # This is where you would implement the logic to send constants to the robot.
        # For now, it just prints the values.
        print("--- Sending Constants ---")
        for const, entry in self.entries.items():
            print(f"{const}: {entry.get()}")
        print("-------------------------")
        # TODO: Implement HTTP POST request to a new robot endpoint, e.g., /jules/set-constants

    def update_data(self):
        # Update labels with data from either the real client or fake generator
        for key, label in self.labels.items():
            value = self.data.get(key, 0.0)
            if key == 'heading':
                 label.config(text=f"{math.degrees(value):.2f}° ({value:.3f} rad)")
            else:
                label.config(text=f"{value:.3f}")
        
        self.root.after(50, self.update_data) # Schedule next update

    def on_closing(self):
        if self.client:
            self.client.stop()
        self.stop_fake_data_thread = True
        if self.fake_data_thread:
            self.fake_data_thread.join()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = JulesTelemetryGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
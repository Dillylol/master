# gui.py

import tkinter as tk
from tkinter import ttk, font, filedialog, messagebox
import math
import requests # <-- Add import
import json     # <-- Add import
from data_stream import JulesClient, FakeDataGenerator
from metrics_datamodel import MetricsData

class JulesTelemetryGUI:
    # ... __init__ is the same ...
    def __init__(self, root):
        self.root = root
        self.root.title("JULES Telemetry Dashboard")
        self.root.geometry("800x600")

        self.data_source = None
        self.use_fake_data = tk.BooleanVar(value=False)

        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.bold_font = font.Font(family="Helvetica", size=10, weight="bold")
        self.mono_font = font.Font(family="Courier", size=10)
        self.style.configure("TLabel", padding=5, font=('Helvetica', 10))
        self.style.configure("Bold.TLabel", padding=5, font=self.bold_font)

        self.labels = {}
        self.entries = {}
        self.create_widgets()
        self.update_gui()


    def create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=5)
        
        # ... IP, Token, Connect Button ...
        ttk.Label(conn_frame, text="IP Address:").grid(row=0, column=0, sticky="w")
        self.ip_entry = ttk.Entry(conn_frame, width=20)
        self.ip_entry.insert(0, "http://192.168.43.1:58080")
        self.ip_entry.grid(row=0, column=1, padx=5)

        ttk.Label(conn_frame, text="Token:").grid(row=0, column=2, sticky="w", padx=10)
        self.token_entry = ttk.Entry(conn_frame, width=25)
        self.token_entry.grid(row=0, column=3, padx=5)

        self.connect_button = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=0, column=4, padx=10)
        
        # --- Add Dump Data Button ---
        self.dump_button = ttk.Button(conn_frame, text="Dump Data to File", command=self.dump_data_to_file)
        self.dump_button.grid(row=0, column=5, padx=10)
        # ---------------------------

        self.status_label = ttk.Label(conn_frame, text="Status: Disconnected", foreground="red")
        self.status_label.grid(row=0, column=6, padx=10)
        
        self.fake_data_check = ttk.Checkbutton(
            conn_frame, text="Use Fake Data", variable=self.use_fake_data, command=self.toggle_data_source)
        self.fake_data_check.grid(row=0, column=7, padx=20)

        # ... Rest of create_widgets is the same ...
        data_frame = ttk.Frame(main_frame)
        data_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 5))
        constants_frame = ttk.LabelFrame(main_frame, text="Pedro Pathing Constants", padding="10")
        constants_frame.grid(row=1, column=1, sticky="nsew", padx=(5, 0))
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        self._create_telemetry_section(data_frame, "Core Metrics", ["t", "cmdPower", "velIPS", "headingDeg", "batteryV"])
        self._create_telemetry_section(data_frame, "Odometry", ["x", "y", "heading"])
        self._create_telemetry_section(data_frame, "IMU", ["pitch", "roll", "yawRate", "pitchRate", "rollRate"])
        self._create_constants_section(constants_frame, 
            ["MASS", "WHEEL_RADIUS", "GEAR_RATIO", "TRACK_WIDTH", "LATERAL_MULTIPLIER"],
            ["10.0", "1.88976", "1.0", "15.5", "1.0"])


    # ... other methods are the same ...
    def _create_telemetry_section(self, parent, title, metrics):
        frame = ttk.LabelFrame(parent, text=title, padding="10")
        frame.pack(fill=tk.X, pady=5, padx=5)
        for i, metric in enumerate(metrics):
            ttk.Label(frame, text=f"{metric}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w")
            self.labels[metric] = ttk.Label(frame, text="0.000", width=20, anchor="w", font=self.mono_font)
            self.labels[metric].grid(row=i, column=1, sticky="w")

    def _create_constants_section(self, parent, constants, defaults):
        for i, const in enumerate(constants):
            ttk.Label(parent, text=f"{const}:", style="Bold.TLabel").grid(row=i, column=0, sticky="w", pady=2)
            self.entries[const] = ttk.Entry(parent, width=15, font=self.mono_font)
            self.entries[const].insert(0, defaults[i])
            self.entries[const].grid(row=i, column=1, sticky="w", padx=5)
        send_button = ttk.Button(parent, text="Send to Robot", command=self.send_constants)
        send_button.grid(row=len(constants), column=0, columnspan=2, pady=10)

    def toggle_data_source(self):
        if self.data_source:
            self.data_source.stop()
            self.data_source = None
        if self.use_fake_data.get():
            self.data_source = FakeDataGenerator(self.on_data_received, self.on_error)
            self.data_source.start()
            self.status_label.config(text="Status: Using Fake Data", foreground="blue")
            self.connect_button.config(text="Connect", state="disabled")
        else:
            self.status_label.config(text="Status: Disconnected", foreground="red")
            self.connect_button.config(text="Connect", state="normal")

    def toggle_connection(self):
        if self.data_source and self.data_source.running:
            self.data_source.stop()
            self.data_source = None
            self.connect_button.config(text="Connect")
            self.status_label.config(text="Status: Disconnected", foreground="red")
        else:
            ip = self.ip_entry.get()
            token = self.token_entry.get()
            self.data_source = JulesClient(ip, token, self.on_data_received, self.on_error)
            self.data_source.start()
            self.connect_button.config(text="Disconnect")
            self.status_label.config(text="Status: Connecting...", foreground="orange")

    def on_data_received(self, data):
        if not self.use_fake_data.get() and self.data_source and self.data_source.running:
             self.status_label.config(text="Status: Connected", foreground="green")

    def on_error(self, error_message):
        if not self.use_fake_data.get():
            self.status_label.config(text=f"Status: Error", foreground="red")
        print(error_message)

    def send_constants(self):
        print("--- Sending Constants ---")
        for const, entry in self.entries.items():
            print(f"{const}: {entry.get()}")

    def update_gui(self):
        if self.data_source:
            data = self.data_source.get_latest_data()
            self.labels['t'].config(text=f"{data.t:.3f}")
            self.labels['cmdPower'].config(text=f"{data.cmdPower:.3f}")
            self.labels['velIPS'].config(text=f"{data.velIPS:.3f}")
            self.labels['headingDeg'].config(text=f"{data.headingDeg:.3f}")
            self.labels['batteryV'].config(text=f"{data.batteryV:.3f}")
            self.labels['x'].config(text=f"{data.x:.3f}")
            self.labels['y'].config(text=f"{data.y:.3f}")
            self.labels['pitch'].config(text=f"{data.pitch:.3f}")
            self.labels['roll'].config(text=f"{data.roll:.3f}")
            self.labels['yawRate'].config(text=f"{data.yawRate:.3f}")
            self.labels['pitchRate'].config(text=f"{data.pitchRate:.3f}")
            self.labels['rollRate'].config(text=f"{data.rollRate:.3f}")
            self.labels['heading'].config(text=f"{math.degrees(data.heading):.2f}° ({data.heading:.3f} rad)")
        self.root.after(50, self.update_gui)

    def on_closing(self):
        if self.data_source:
            self.data_source.stop()
        self.root.destroy()
        
    def dump_data_to_file(self):
        """Handle the 'Dump Data' button press."""
        file_path = filedialog.asksaveasfilename(
            defaultextension=".jsonl",
            filetypes=[("JSON Lines", "*.jsonl"), ("All Files", "*.*")],
            title="Save Data Dump"
        )
        if not file_path:
            return # User cancelled

        try:
            if self.use_fake_data.get():
                if self.data_source:
                    # Get buffered fake data and save it
                    buffered_data = self.data_source.get_buffered_data()
                    with open(file_path, 'w') as f:
                        for record in buffered_data:
                            f.write(json.dumps(record) + '\n')
                    messagebox.showinfo("Success", f"Saved {len(buffered_data)} fake data records to {file_path}")
                else:
                    messagebox.showerror("Error", "Fake data generator not running.")
            
            else: # Real data from robot
                ip = self.ip_entry.get()
                token = self.token_entry.get()
                if not ip or not token:
                    messagebox.showerror("Error", "IP and Token required to dump data.")
                    return

                # Make request to the dump endpoint
                url = f"{ip.rstrip('/')}/jules/dump?token={token}"
                response = requests.get(url)
                response.raise_for_status() # Raise an exception for bad status codes

                # Save the response content to the file
                with open(file_path, 'w') as f:
                    f.write(response.text)
                
                num_lines = len(response.text.splitlines())
                messagebox.showinfo("Success", f"Saved {num_lines} records from robot to {file_path}")

        except Exception as e:
            messagebox.showerror("Error Dumping Data", f"An error occurred: {e}")
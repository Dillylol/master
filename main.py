# main.py

import tkinter as tk
from gui import JulesTelemetryGUI

if __name__ == "__main__":
    root = tk.Tk()
    app = JulesTelemetryGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import serial
import time

class LivePlotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Torques and Position Data over Time")
        self.root.geometry("1200x900")

        # Serial setup
        try:
            self.ser = serial.Serial('COM13', 115200, timeout=1)
            time.sleep(2)
            print("Serial connection established.")
        except Exception as e:
            print(f"Serial connection failed: {e}")
            self.ser = None

        # Create figure with 2 subplots
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Input fields
        self.input_frame = tk.Frame(self.root)
        self.input_frame.pack(pady=10)

        tk.Label(self.input_frame, text="Splay Angle:").grid(row=0, column=0)
        self.splay_entry = tk.Entry(self.input_frame, width=10)
        self.splay_entry.insert(0, "0.0")
        self.splay_entry.grid(row=0, column=1)

        tk.Label(self.input_frame, text="Proximal Angle:").grid(row=0, column=2)
        self.proximal_entry = tk.Entry(self.input_frame, width=10)
        self.proximal_entry.insert(0, "0.6")
        self.proximal_entry.grid(row=0, column=3)

        tk.Label(self.input_frame, text="Distal Angle:").grid(row=0, column=4)
        self.distal_entry = tk.Entry(self.input_frame, width=10)
        self.distal_entry.insert(0, "0.6")
        self.distal_entry.grid(row=0, column=5)

        self.set_button = tk.Button(self.input_frame, text="Apply", command=self.apply_inputs)
        self.set_button.grid(row=0, column=6, padx=10)

        # Angle variables
        self.splay_angle = 0.0
        self.proximal_angle = 0.6
        self.distal_angle = 0.6

        # Data tracking
        self.start_time = time.time()
        self.x_data = []
        self.y_data = [[] for _ in range(12)]  # 12 channels total

        # Motor torque lines (subplot 1)
        torque_colors = ['red', 'orange', 'green', 'blue']
        torque_labels = ['Motor 0', 'Motor 1', 'Motor 2', 'Motor 3']
        self.lines1 = []
        for i in range(4):
            line, = self.ax1.plot([], [], color=torque_colors[i], label=torque_labels[i])
            self.lines1.append(line)

        self.ax1.set_ylabel("Motor Torques (Nm)")
        self.ax1.legend(loc="upper left")

        # Extra data lines (subplot 2)
        extra_colors = ['red', 'orange', 'green', 'blue', 'red', 'orange', 'green', 'blue']
        extra_labels = [f"Extra {i}" for i in range(8)]
        self.lines2 = []
        for i in range(8):
            style = '-' if i < 4 else ':'  # last 4 dotted
            line, = self.ax2.plot([], [], color=extra_colors[i], linestyle=style, label=extra_labels[i])
            self.lines2.append(line)

        self.ax2.set_ylabel("Position")
        self.ax2.set_xlabel("Time (s)")
        self.ax2.legend(loc="upper left")

        self.update_plot()

    def apply_inputs(self):
        try:
            self.splay_angle = float(self.splay_entry.get())
            self.proximal_angle = float(self.proximal_entry.get())
            self.distal_angle = float(self.distal_entry.get())
            print(f"Applied angles: Splay={self.splay_angle}, Proximal={self.proximal_angle}, Distal={self.distal_angle}")

            # Send to Teensy if serial is open
            if self.ser and self.ser.is_open:
                msg = f"{self.splay_angle},{self.proximal_angle},{self.distal_angle}\n"
                self.ser.write(msg.encode())
        except ValueError:
            print("Invalid input.")

    def read_teensy_data(self):
        if self.ser:
            latest_data = None
            while self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line.startswith("DATA"):
                        values = list(map(float, line.split(",")[1:]))
                        if len(values) == 12:
                            latest_data = values
                except Exception:
                    pass
            return latest_data
        return None

    def update_plot(self):
        elapsed = time.time() - self.start_time

        # Read from Teensy or fallback to zeros
        data = self.read_teensy_data()
        if not data:
            data = [0.0] * 12

        if len(self.x_data) >= 100:
            self.x_data.pop(0)
            for y in self.y_data:
                y.pop(0)

        self.x_data.append(elapsed)
        for i in range(12):
            self.y_data[i].append(data[i])

        # Update torque plot
        for i in range(4):
            self.lines1[i].set_data(self.x_data, self.y_data[i])

        # Update extra data plot
        for i in range(8):
            self.lines2[i].set_data(self.x_data, self.y_data[i + 4])

        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.canvas.draw()

        self.root.after(100, self.update_plot)

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")

# Run the app
if __name__ == "__main__":
    root = tk.Tk()
    app = LivePlotApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close_serial(), root.destroy()))
    root.mainloop()

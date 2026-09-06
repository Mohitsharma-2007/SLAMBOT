import sys
import os
import subprocess
import threading
import queue
import time
import json
import tkinter as tk
from tkinter import messagebox, scrolledtext
from tkinter import ttk
import httpx

# Theme Palette (Dark Mode)
BG_MAIN = "#121214"
BG_PANEL = "#1a1a1e"
BG_PANEL_ALT = "#24242b"
FG_TEXT = "#e1e1e6"
FG_TEXT_DIM = "#8d8d99"
ACCENT_GREEN = "#00e676"
ACCENT_BLUE = "#00b0ff"
ACCENT_RED = "#ff5252"
ACCENT_ORANGE = "#ff9100"
BORDER_COLOR = "#2a2a30"

class SlamBotDiagnosticsApp:
    def __init__(self, root):
        self.root = root
        self.root.title("SLAM Bot — Diagnostic Console & Process Manager")
        self.root.geometry("1200x800")
        self.root.configure(bg=BG_MAIN)

        # Subprocess variables
        self.backend_proc = None
        self.frontend_proc = None
        
        # Thread queues for logs
        self.backend_queue = queue.Queue()
        self.frontend_queue = queue.Queue()
        
        # Connection polling state
        self.running_poll = True
        self.last_arduino_msg_count = 0
        self.last_nodemcu_msg_count = 0
        self.last_poll_time = time.time()
        self.power_history = [0.0] * 50
        
        # Configure Grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.setup_styles()
        self.build_ui()
        
        # Start log reading timers
        self.root.after(100, self.process_backend_logs)
        self.root.after(100, self.process_frontend_logs)
        
        # Start API polling thread
        self.poll_thread = threading.Thread(target=self.poll_api_loop, daemon=True)
        self.poll_thread.start()
        
        # Clean shutdown on window close
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", bg=BG_MAIN, fg=FG_TEXT)
        style.configure("TFrame", background=BG_MAIN)
        style.configure("TLabel", background=BG_MAIN, foreground=FG_TEXT, font=("Segoe UI", 10))
        style.configure("Panel.TFrame", background=BG_PANEL, relief="flat")
        
        # Buttons
        style.configure("TButton", font=("Segoe UI", 10, "bold"), background=BG_PANEL_ALT, foreground=FG_TEXT, borderwidth=1, focuscolor="")
        style.map("TButton",
                  background=[("active", ACCENT_BLUE), ("disabled", "#323238")],
                  foreground=[("active", "#000000"), ("disabled", "#5c5c64")])
                  
        style.configure("Start.TButton", background=ACCENT_GREEN, foreground="#000000")
        style.map("Start.TButton", background=[("active", "#66ffa6")])
        
        style.configure("Stop.TButton", background=ACCENT_RED, foreground=FG_TEXT)
        style.map("Stop.TButton", background=[("active", "#ff867f")])

    def build_ui(self):
        # Main layout splitter
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # =====================================================================
        # LEFT COLUMN (Controls, Telemetry, Power, Diagnostic Report)
        # =====================================================================
        left_frame = ttk.Frame(main_paned, style="TFrame")
        main_paned.add(left_frame, weight=1)
        
        # 1. Process Controllers
        proc_box = ttk.LabelFrame(left_frame, text=" Process Manager ", class_="Panel.TFrame")
        proc_box.pack(fill=tk.X, pady=(0, 10))
        
        btn_grid = tk.Frame(proc_box, bg=BG_PANEL)
        btn_grid.pack(padx=10, pady=10, fill=tk.X)
        
        self.btn_start_backend = ttk.Button(btn_grid, text="Start Backend", command=self.start_backend)
        self.btn_start_backend.grid(row=0, column=0, padx=5, pady=5, sticky="ew")
        
        self.btn_stop_backend = ttk.Button(btn_grid, text="Stop Backend", command=self.stop_backend, state=tk.DISABLED)
        self.btn_stop_backend.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.btn_start_frontend = ttk.Button(btn_grid, text="Start Frontend", command=self.start_frontend)
        self.btn_start_frontend.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        self.btn_stop_frontend = ttk.Button(btn_grid, text="Stop Frontend", command=self.stop_frontend, state=tk.DISABLED)
        self.btn_stop_frontend.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)

        # 2. Connection Status (LEDs & Telemetry)
        conn_box = ttk.LabelFrame(left_frame, text=" Live Telemetry & Connections ", class_="Panel.TFrame")
        conn_box.pack(fill=tk.X, pady=10)
        
        # Status Canvas (LEDs)
        self.status_canvas = tk.Canvas(conn_box, width=320, height=80, bg=BG_PANEL, highlightthickness=0)
        self.status_canvas.pack(pady=5, padx=10, fill=tk.X)
        self.draw_status_leds()
        
        # Stats Labels
        stats_frame = tk.Frame(conn_box, bg=BG_PANEL)
        stats_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.lbl_arduino_net = tk.Label(stats_frame, text="Arduino Link: Offline (0 Hz)", bg=BG_PANEL, fg=FG_TEXT_DIM, anchor="w", font=("Segoe UI", 9))
        self.lbl_arduino_net.pack(fill=tk.X, pady=2)
        
        self.lbl_nodemcu_net = tk.Label(stats_frame, text="NodeMCU Link: Offline (0 Hz)", bg=BG_PANEL, fg=FG_TEXT_DIM, anchor="w", font=("Segoe UI", 9))
        self.lbl_nodemcu_net.pack(fill=tk.X, pady=2)
        
        self.lbl_bandwidth = tk.Label(stats_frame, text="Net Bandwidth: 0.0 KB/s | Latency: -- ms", bg=BG_PANEL, fg=FG_TEXT_DIM, anchor="w", font=("Segoe UI", 9))
        self.lbl_bandwidth.pack(fill=tk.X, pady=2)

        # 3. Power Estimator & Live Graph
        power_box = ttk.LabelFrame(left_frame, text=" Estimated Power consumption ", class_="Panel.TFrame")
        power_box.pack(fill=tk.X, pady=10)
        
        self.lbl_power_text = tk.Label(power_box, text="Current Draw: -- mA | Power: -- Watts | Battery: Estimating...", bg=BG_PANEL, fg=FG_TEXT, font=("Segoe UI", 10, "bold"), anchor="w")
        self.lbl_power_text.pack(padx=10, pady=5, fill=tk.X)
        
        self.graph_canvas = tk.Canvas(power_box, height=100, bg="#121214", highlightthickness=1, highlightbackground=BORDER_COLOR)
        self.graph_canvas.pack(fill=tk.X, padx=10, pady=(5, 10))
        self.draw_power_graph()

        # 4. Diagnostics & Decision Engine (Why bot isn't moving)
        diag_box = ttk.LabelFrame(left_frame, text=" Autonomous Navigation Debugger & Report ", class_="Panel.TFrame")
        diag_box.pack(fill=tk.BOTH, expand=True, pady=10)
        
        self.diag_text = tk.Text(diag_box, wrap=tk.WORD, height=10, bg="#121214", fg=FG_TEXT, insertbackground=FG_TEXT, relief="flat", highlightthickness=1, highlightbackground=BORDER_COLOR, font=("Consolas", 9))
        self.diag_text.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.diag_text.insert(tk.END, "Diagnostic Engine Idle. Start the backend and load the robot to begin analysis.")
        self.diag_text.config(state=tk.DISABLED)

        # =====================================================================
        # RIGHT COLUMN (Log outputs)
        # =====================================================================
        right_frame = ttk.Frame(main_paned, style="TFrame")
        main_paned.add(right_frame, weight=2)
        
        right_paned = ttk.PanedWindow(right_frame, orient=tk.VERTICAL)
        right_paned.pack(fill=tk.BOTH, expand=True)
        
        # Backend Log Terminal
        backend_log_box = ttk.LabelFrame(right_paned, text=" FastAPI Backend Logs (Port 8000) ", class_="Panel.TFrame")
        right_paned.add(backend_log_box, weight=1)
        
        self.backend_log_text = scrolledtext.ScrolledText(backend_log_box, bg="#0d0d0f", fg=FG_TEXT, insertbackground=FG_TEXT, font=("Consolas", 9), relief="flat")
        self.backend_log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Frontend Log Terminal
        frontend_log_box = ttk.LabelFrame(right_paned, text=" Vite Frontend Logs (Port 5173) ", class_="Panel.TFrame")
        right_paned.add(frontend_log_box, weight=1)
        
        self.frontend_log_text = scrolledtext.ScrolledText(frontend_log_box, bg="#0d0d0f", fg=FG_TEXT, insertbackground=FG_TEXT, font=("Consolas", 9), relief="flat")
        self.frontend_log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def draw_status_leds(self, backend=False, arduino=False, nodemcu=False, ros=False):
        self.status_canvas.delete("all")
        
        # 4 LEDs configuration
        leds = [
            ("Backend", backend, 25),
            ("Arduino", arduino, 105),
            ("NodeMCU", nodemcu, 185),
            ("ROS2 (WSL)", ros, 265)
        ]
        
        for name, state, x in leds:
            color = ACCENT_GREEN if state else ACCENT_RED
            shadow = "#006400" if state else "#8b0000"
            # Outer Ring
            self.status_canvas.create_oval(x-20, 10, x+20, 50, fill="#2a2a30", outline=BORDER_COLOR, width=2)
            # Glowing core
            self.status_canvas.create_oval(x-15, 15, x+15, 45, fill=color, outline=shadow, width=1)
            # Gloss shine highlight
            self.status_canvas.create_oval(x-8, 18, x-2, 24, fill="#ffffff", outline="")
            # Text label
            self.status_canvas.create_text(x, 65, text=name, fill=FG_TEXT, font=("Segoe UI", 8, "bold"))

    def draw_power_graph(self):
        self.graph_canvas.delete("all")
        w = self.graph_canvas.winfo_width()
        h = self.graph_canvas.winfo_height()
        if w < 10: w = 300
        if h < 10: h = 100
        
        # Draw background grids
        for i in range(1, 4):
            y = h - (h * i // 4)
            self.graph_canvas.create_line(0, y, w, y, fill="#1c1c22", dash=(2, 2))
            # Grid text
            power_val = f"{i * 2.5:.1f}W"
            self.graph_canvas.create_text(15, y - 8, text=power_val, fill=FG_TEXT_DIM, font=("Segoe UI", 7))

        # Plot history lines
        coords = []
        n = len(self.power_history)
        max_power = 10.0 # scale to 10 Watts max
        for i, val in enumerate(self.power_history):
            x = (w * i) / (n - 1)
            # clamp to max_power
            val_clamped = min(val, max_power)
            y = h - (h * val_clamped / max_power)
            coords.append((x, y))
            
        for i in range(len(coords) - 1):
            x1, y1 = coords[i]
            x2, y2 = coords[i+1]
            self.graph_canvas.create_line(x1, y1, x2, y2, fill=ACCENT_BLUE, width=2)

    # =====================================================================
    # PROCESS MANAGEMENT (Start / Stop)
    # =====================================================================
    def start_backend(self):
        self.btn_start_backend.config(state=tk.DISABLED)
        self.btn_stop_backend.config(state=tk.NORMAL)
        
        # Run using same python interpreter running this script
        cmd = [sys.executable, "backend/main.py"]
        
        # Environment configuration
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        
        try:
            self.backend_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
                bufsize=1
            )
            # Thread to read output
            t = threading.Thread(target=self.enqueue_output, args=(self.backend_proc, self.backend_queue), daemon=True)
            t.start()
            self.log_backend("[Diag GUI] FastAPI backend started.\n")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start backend: {e}")
            self.btn_start_backend.config(state=tk.NORMAL)
            self.btn_stop_backend.config(state=tk.DISABLED)

    def stop_backend(self):
        if self.backend_proc:
            self.backend_proc.terminate()
            self.backend_proc = None
        self.btn_start_backend.config(state=tk.NORMAL)
        self.btn_stop_backend.config(state=tk.DISABLED)
        self.log_backend("[Diag GUI] FastAPI backend stopped.\n")
        self.draw_status_leds(backend=False)

    def start_frontend(self):
        self.btn_start_frontend.config(state=tk.DISABLED)
        self.btn_stop_frontend.config(state=tk.NORMAL)
        
        # npm run dev requires shell=True on Windows
        try:
            self.frontend_proc = subprocess.Popen(
                "npm run dev",
                cwd="webapp",
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                shell=True,
                bufsize=1
            )
            t = threading.Thread(target=self.enqueue_output, args=(self.frontend_proc, self.frontend_queue), daemon=True)
            t.start()
            self.log_frontend("[Diag GUI] Vite WebApp dev server started.\n")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to start frontend: {e}")
            self.btn_start_frontend.config(state=tk.NORMAL)
            self.btn_stop_frontend.config(state=tk.DISABLED)

    def stop_frontend(self):
        if self.frontend_proc:
            # On Windows, terminating shell subprocesses requires taskkill to kill the child tree
            subprocess.run(f"taskkill /F /T /PID {self.frontend_proc.pid}", stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.frontend_proc = None
        self.btn_start_frontend.config(state=tk.NORMAL)
        self.btn_stop_frontend.config(state=tk.DISABLED)
        self.log_frontend("[Diag GUI] Vite WebApp dev server stopped.\n")

    # =====================================================================
    # LOG READING THREADS
    # =====================================================================
    def enqueue_output(self, proc, q):
        for line in iter(proc.stdout.readline, ''):
            q.put(line)
        proc.stdout.close()

    def process_backend_logs(self):
        while not self.backend_queue.empty():
            line = self.backend_queue.get_nowait()
            self.log_backend(line)
        self.root.after(100, self.process_backend_logs)

    def process_frontend_logs(self):
        while not self.frontend_queue.empty():
            line = self.frontend_queue.get_nowait()
            self.log_frontend(line)
        self.root.after(100, self.process_frontend_logs)

    def log_backend(self, text):
        self.backend_log_text.config(state=tk.NORMAL)
        self.backend_log_text.insert(tk.END, text)
        self.backend_log_text.see(tk.END)
        # Limit text size
        if float(self.backend_log_text.index('end-1c')) > 5000.0:
            self.backend_log_text.delete('1.0', '500.0')
        self.backend_log_text.config(state=tk.DISABLED)

    def log_frontend(self, text):
        self.frontend_log_text.config(state=tk.NORMAL)
        self.frontend_log_text.insert(tk.END, text)
        self.frontend_log_text.see(tk.END)
        if float(self.frontend_log_text.index('end-1c')) > 5000.0:
            self.frontend_log_text.delete('1.0', '500.0')
        self.frontend_log_text.config(state=tk.DISABLED)

    # =====================================================================
    # API POLLING & TELEMETRY MONITOR
    # =====================================================================
    def poll_api_loop(self):
        while self.running_poll:
            time.sleep(0.5) # Poll status at 2 Hz
            now = time.time()
            dt = now - self.last_poll_time
            self.last_poll_time = now

            try:
                # Query status
                with httpx.Client(timeout=0.4) as client:
                    r = client.get("http://localhost:8000/api/status")
                
                if r.status_code == 200:
                    status = r.json()
                    self.root.after(0, self.update_gui_telemetry, status, dt)
                else:
                    self.root.after(0, self.update_gui_offline, f"HTTP Error {r.status_code}")
            except Exception as e:
                self.root.after(0, self.update_gui_offline, str(e))

    def update_gui_telemetry(self, status, dt):
        # 1. Update Connection LEDs
        backend_on = True
        
        ard_info = status.get("devices", {}).get("arduino", {})
        arduino_on = ard_info.get("connected", False)
        
        nodemcu_info = status.get("devices", {}).get("nodemcu", {})
        nodemcu_on = nodemcu_info.get("connected", False)
        
        ros_info = status.get("ros", {})
        ros_on = ros_info.get("available", False)
        
        self.draw_status_leds(backend=backend_on, arduino=arduino_on, nodemcu=nodemcu_on, ros=ros_on)

        # 2. Calculate speeds (Hz) and Packet Info
        ard_msgs = ard_info.get("messages_received", 0)
        nodemcu_msgs = nodemcu_info.get("messages_received", 0)
        
        ard_hz = max(0.0, (ard_msgs - self.last_arduino_msg_count) / dt) if self.last_arduino_msg_count > 0 else 0.0
        nodemcu_hz = max(0.0, (nodemcu_msgs - self.last_nodemcu_msg_count) / dt) if self.last_nodemcu_msg_count > 0 else 0.0
        
        self.last_arduino_msg_count = ard_msgs
        self.last_nodemcu_msg_count = nodemcu_msgs
        
        # Estimate Bandwidth (Arduino ~220B per frame, NodeMCU ~2.5KB per scan)
        bandwidth_bytes = (ard_hz * 220.0) + (nodemcu_hz * 2500.0)
        bandwidth_kb_s = bandwidth_bytes / 1024.0
        
        ard_stale = ard_info.get("stale_ms")
        if ard_stale is None:
            ard_stale = 0
        nodemcu_stale = nodemcu_info.get("stale_ms")
        if nodemcu_stale is None:
            nodemcu_stale = 0
        latency = max(ard_stale, nodemcu_stale)
        
        self.lbl_arduino_net.config(
            text=f"Arduino Link: Connected ({ard_info.get('ip', 'Unknown')}) | Rate: {ard_hz:.1f} Hz (Total: {ard_msgs})", 
            fg=ACCENT_GREEN
        )
        self.lbl_nodemcu_net.config(
            text=f"NodeMCU Link: Connected ({nodemcu_info.get('ip', 'Unknown')}) | Rate: {nodemcu_hz:.1f} Hz (Total: {nodemcu_msgs})", 
            fg=ACCENT_GREEN
        )
        self.lbl_bandwidth.config(
            text=f"Net Bandwidth: {bandwidth_kb_s:.2f} KB/s | Connection Staleness: {latency} ms"
        )

        # 3. Estimate Power Consumption
        # Idle loads
        power = 1.5 # Arduino R4 WiFi idle
        power += 0.8 # NodeMCU idle
        if nodemcu_on:
            power += 1.2 # RPLIDAR rotating/emitting
            
        # Motor loads (computed dynamically from wheel duty cycle)
        odom = status.get("odom", {})
        duty_l = abs(odom.get("duty_l", 0))
        duty_r = abs(odom.get("duty_r", 0))
        # N20 active power model: 2.5W max per motor at full load
        motor_power = ((duty_l / 255.0) + (duty_r / 255.0)) * 2.5
        total_power = power + motor_power
        
        # Estimated Current at 7.4V nominal 2S Battery
        current_ma = (total_power / 7.4) * 1000.0
        
        self.lbl_power_text.config(
            text=f"Current Draw: {current_ma:.0f} mA | Power: {total_power:.2f} Watts | Source: 7.4V LiPo (Estimated)",
            fg=ACCENT_GREEN
        )
        
        # Update power graph
        self.power_history.pop(0)
        self.power_history.append(total_power)
        self.draw_power_graph()

        # 4. Diagnostic & Decision Tree Engine
        self.generate_diagnostic_report(status, arduino_on, nodemcu_on, ros_on)

    def update_gui_offline(self, err_msg):
        self.draw_status_leds(backend=False)
        self.lbl_arduino_net.config(text="Arduino Link: Offline (0 Hz)", fg=FG_TEXT_DIM)
        self.lbl_nodemcu_net.config(text="NodeMCU Link: Offline (0 Hz)", fg=FG_TEXT_DIM)
        self.lbl_bandwidth.config(text="Net Bandwidth: 0.0 KB/s | Latency: -- ms")
        self.lbl_power_text.config(text="Current Draw: -- mA | Power: -- Watts | Battery: Offline", fg=FG_TEXT_DIM)
        
        self.diag_text.config(state=tk.NORMAL)
        self.diag_text.delete("1.0", tk.END)
        self.diag_text.insert(tk.END, f"[OFFLINE] Backend API is unreachable.\nError detail: {err_msg}\n\nTroubleshooting:\n1. If you haven't started the backend yet, click 'Start Backend' above.\n2. Verify that no other process is bound to port 8000.")
        self.diag_text.config(state=tk.DISABLED)

    def generate_diagnostic_report(self, status, arduino_on, nodemcu_on, ros_on):
        report = []
        report.append("=== SLAM BOT DECISION ENGINE REPORT ===")
        report.append(f"Time: {time.strftime('%H:%M:%S')} | Control Mode: {status.get('control_mode', 'Unknown')}")
        report.append("-" * 50)
        
        # Flags
        running = status.get("running", False)
        estop = status.get("estop_latched", False)
        odom = status.get("odom", {})
        
        # State Checklist
        report.append(f"[*] State RUNNING toggle in WebApp: {'[OK] ACTIVE' if running else '[STOPPED] INACTIVE'}")
        report.append(f"[*] E-STOP state: {'[LATCHED] EMERGENCY STOP ACTIVE' if estop else '[OK] RELEASED'}")
        report.append(f"[*] Arduino Connection: {'[OK] ONLINE' if arduino_on else '[DISCONNECTED] OFFLINE'}")
        report.append(f"[*] NodeMCU LIDAR Connection: {'[OK] ONLINE' if nodemcu_on else '[DISCONNECTED] OFFLINE'}")
        report.append(f"[*] ROS2 / Nav2 Stack in WSL: {'[OK] RUNNING' if ros_on else '[OFFLINE] NOT DETECTED'}")
        
        # Command Analysis
        ticks_l = odom.get("ticks_l", 0)
        ticks_r = odom.get("ticks_r", 0)
        duty_l = odom.get("duty_l", 0)
        duty_r = odom.get("duty_r", 0)
        report.append(f"[*] Motor Outputs: Left Duty: {duty_l} (Ticks: {ticks_l}) | Right Duty: {duty_r} (Ticks: {ticks_r})")
        
        report.append("\n=== DIAGNOSTIC EVALUATION ===")
        
        issues = []
        
        if estop:
            issues.append("- CRITICAL: E-STOP is active! The motors cannot rotate. Click the 'Release E-Stop' button on the WebApp Dashboard.")
        if not running:
            issues.append("- WARNING: The robot is in 'STOP' state in the WebApp. You must toggle it to 'START' to send power to the motors.")
        if not arduino_on:
            issues.append("- WARNING: Arduino is not connected. If it has been flashed, verify it connects to your phone's Wi-Fi. Ensure the phone Hotspot is enabled.")
        if not nodemcu_on:
            issues.append("- WARNING: NodeMCU LIDAR is offline. Map building and obstacle costmaps will not update.")
        if not ros_on:
            issues.append("- WARNING: ROS2 is not running. Autonomous goals clicked on the map will do nothing.\n  Action: Connect to WSL and run: 'bash start_wsl_ros.sh'")
            
        if not issues:
            # Check if navigation command is active
            nav_active = False
            # Check logs/status if nav is sending values
            ros_pose = status.get("ros", {})
            nodes = ros_pose.get("nodes", [])
            report.append(f"Active ROS2 Nodes: {', '.join(nodes) if nodes else 'None'}")
            
            if len(nodes) > 0:
                report.append("\n[OK] All physical systems and WSL bridges are connected.")
                report.append("If the robot does not move autonomously when you click a goal:")
                report.append("1. Open the WebApp map page and check if slam_toolbox has successfully localized. If the map is blank, drive manual CCW once to initialize.")
                report.append("2. Look at the WSL console logs. Ensure Nav2's 'bt_navigator' has finished loading behavior trees.")
                report.append("3. Make sure the robot is in 'START' mode (not STOP).")
            else:
                report.append("\n[IDLE] Systems connected, awaiting manual control or a Nav2 path goal.")
        else:
            report.append("\n".join(issues))
            
        # Draw Report
        self.diag_text.config(state=tk.NORMAL)
        self.diag_text.delete("1.0", tk.END)
        self.diag_text.insert(tk.END, "\n".join(report))
        self.diag_text.config(state=tk.DISABLED)

    # =====================================================================
    # SHUTDOWN
    # =====================================================================
    def on_close(self):
        self.running_poll = False
        self.stop_backend()
        self.stop_frontend()
        self.root.destroy()

if __name__ == "__main__":
    # Ensure Tkinter runs correctly
    root = tk.Tk()
    app = SlamBotDiagnosticsApp(root)
    root.mainloop()

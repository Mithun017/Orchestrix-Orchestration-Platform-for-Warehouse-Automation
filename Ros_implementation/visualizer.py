import tkinter as tk
import requests
import time
import threading
import math

# Configuration
WCS_URL = "http://127.0.0.1:8000"
GRID_SIZE = 10
CELL_SIZE = 60
UPDATE_RATE_MS = 50  # Faster refresh for smooth animation

# Colors
COLOR_BG = "#1e1e1e"
COLOR_GRID = "#333333"
COLOR_OBSTACLE = "#475569"
COLOR_ROBOT_BUSY = "#10b981"
COLOR_ROBOT_IDLE = "#94a3b8"
COLOR_PATH = "#10b981"

class OrchestrixViz:
    def __init__(self, root):
        self.root = root
        self.root.title("ORCHESTRIX SIMULATION (RViz-Lite)")
        self.root.geometry(f"{GRID_SIZE * CELL_SIZE + 40}x{GRID_SIZE * CELL_SIZE + 80}")
        self.root.configure(bg=COLOR_BG)

        # Header
        self.header = tk.Label(root, text="ORCHESTRIX: A* Path Planning Viz", fg="white", bg=COLOR_BG, font=("Segoe UI", 12, "bold"))
        self.header.pack(pady=5)

        # Canvas
        self.canvas = tk.Canvas(root, width=GRID_SIZE * CELL_SIZE, height=GRID_SIZE * CELL_SIZE, bg="#0f172a", highlightthickness=1, highlightbackground=COLOR_GRID)
        self.canvas.pack(pady=0)

        self.status_label = tk.Label(root, text="Connecting to Core...", fg="#fbbf24", bg=COLOR_BG, font=("Consolas", 10))
        self.status_label.pack(pady=5)

        # State
        self.obstacles = []
        self.robots = [] # List of dicts
        self.robot_visuals = {} # Map robot_id -> {x, y, target_x, target_y} for interpolation
        self.running = True

        # Start Polling Thread
        self.poll_thread = threading.Thread(target=self.poll_backend, daemon=True)
        self.poll_thread.start()

        # Start Rendering Loop
        self.render_loop()

    def poll_backend(self):
        while self.running:
            try:
                # 1. Fetch Robots
                r_resp = requests.get(f"{WCS_URL}/wcs/robots", timeout=1)
                if r_resp.status_code == 200:
                    self.robots = r_resp.json()
                    self.status_label.config(text=f"SYSTEM ONLINE | Fleet: {len(self.robots)} | Mode: SIMULATION", fg="#10b981")
                
                # 2. Fetch Map
                m_resp = requests.get(f"{WCS_URL}/map/obstacles", timeout=1)
                if m_resp.status_code == 200:
                    self.obstacles = m_resp.json() # List of [x, y]

                time.sleep(0.5) # Poll backend at 2Hz
            except Exception:
                self.status_label.config(text="DISCONNECTED - Start 'start_orchestrator.bat'", fg="#ef4444")
                time.sleep(2)

    def draw_grid_background(self):
        self.canvas.delete("bg") # Clear background items
        
        # Grid Lines
        for i in range(GRID_SIZE + 1):
            p = i * CELL_SIZE
            self.canvas.create_line(0, p, GRID_SIZE * CELL_SIZE, p, fill=COLOR_GRID, width=1, tags="bg")
            self.canvas.create_line(p, 0, p, GRID_SIZE * CELL_SIZE, fill=COLOR_GRID, width=1, tags="bg")

        # Obstacles
        for obs in self.obstacles:
            x, y = obs[0] * CELL_SIZE, obs[1] * CELL_SIZE
            self.canvas.create_rectangle(x+2, y+2, x+CELL_SIZE-2, y+CELL_SIZE-2, fill=COLOR_OBSTACLE, outline="", tags="bg")

        # Static Locations (Hardcoded for visuals as per backend)
        locations = {
            (0,0): ("RX", "#059669"),   # Receiving
            (9,9): ("TX", "#2563eb"),   # Shipping
            (5,5): ("CHG", "#4f46e5"),  # Charging
        }
        for (gx, gy), (name, color) in locations.items():
            x, y = gx * CELL_SIZE, gy * CELL_SIZE
            self.canvas.create_rectangle(x+4, y+4, x+CELL_SIZE-4, y+CELL_SIZE-4, outline=color, width=2, tags="bg")
            self.canvas.create_text(x+CELL_SIZE/2, y+CELL_SIZE/2, text=name, fill=color, font=("Segoe UI", 9, "bold"), tags="bg")

    def lerp(self, start, end, alpha):
        return start + (end - start) * alpha

    def draw_scene(self):
        self.canvas.delete("all")
        self.draw_grid_background()

        for r in self.robots:
            rid = r['robot_id']
            target_x = r['x']
            target_y = r['y']

            # Interpolation Logic
            if rid not in self.robot_visuals:
                self.robot_visuals[rid] = {'x': target_x, 'y': target_y}
            
            curr = self.robot_visuals[rid]
            # Move towards target (Smooth Lerp)
            curr['x'] = self.lerp(curr['x'], target_x, 0.2) 
            curr['y'] = self.lerp(curr['y'], target_y, 0.2)
            
            screen_x = curr['x'] * CELL_SIZE + CELL_SIZE/2
            screen_y = curr['y'] * CELL_SIZE + CELL_SIZE/2

            # 1. Draw Path (A* PLAN)
            if 'path' in r and r['path'] and len(r['path']) > 0:
                coords = [(screen_x, screen_y)] # Start at current interp pos
                for pt in r['path']:
                     coords.append((pt[0] * CELL_SIZE + CELL_SIZE/2, pt[1] * CELL_SIZE + CELL_SIZE/2))
                
                if len(coords) > 1:
                    self.canvas.create_line(coords, fill=COLOR_PATH, width=3, dash=(4, 2), capstyle=tk.ROUND)

            # 2. Draw Robot Body
            color = COLOR_ROBOT_BUSY if r['status'] == 'BUSY' else COLOR_ROBOT_IDLE
            r_size = 22
            
            # Pulse Effect (Simple Halo)
            if r['status'] == 'BUSY':
                 self.canvas.create_oval(screen_x-r_size-4, screen_y-r_size-4, screen_x+r_size+4, screen_y+r_size+4, outline=color, width=1)

            self.canvas.create_oval(screen_x-r_size, screen_y-r_size, screen_x+r_size, screen_y+r_size, fill=color, outline="white", width=2)
            self.canvas.create_text(screen_x, screen_y, text="R", fill="white", font=("Segoe UI", 10, "bold"))
            self.canvas.create_text(screen_x, screen_y-30, text=rid, fill="white", font=("Segoe UI", 8))

    def render_loop(self):
        if self.running:
            self.draw_scene()
            self.root.after(UPDATE_RATE_MS, self.render_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = OrchestrixViz(root)
    root.mainloop()

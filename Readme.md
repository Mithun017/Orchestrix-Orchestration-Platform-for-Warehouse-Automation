# ORCHESTRIX - Intelligent Warehouse Orchestration Platform

## 🚀 Overview
**ORCHESTRIX** is a cutting-edge simulation platform designed to orchestrate a fleet of Autonomous Mobile Robots (AMRs) within a dynamic warehouse environment. It bridges the gap between high-level business logic (WMS) and low-level robot execution (RCS).

The platform solves the **Multi-Agent Path Finding (MAPF)** problem in dynamic environments, ensuring robots can navigate efficiently without collisions, even when obstacles appear suddenly.

---

## 🏗 Architecture
The system follows a standard 3-layer industrial architecture:

1.  **WMS (Warehouse Management System)**: 
    -   High-level "Business Brain".
    -   Manages inventory and generates tasks (e.g., "Move Pallet #101 from Receiving to Storage").
    -   *Tech*: Python/FastAPI.

2.  **WCS (Warehouse Control System)**: 
    -   The "Traffic Controller".
    -   Receives tasks from WMS and assigns them to the optimal available robot.
    -   Monitors fleet status and battery levels.
    -   *Tech*: Python/FastAPI.

3.  **RCS (Robot Control System)**: 
    -   The "Driver".
    -   Calculates paths, controls movement, and handles local obstacle avoidance.
    -   *Tech*: Python (A* Implementation).
    -   *Optional Extension*: ROS2 Integration available in `Ros_implementation`.

---

## 🧠 Algorithms Used

### 1. A* Pathfinding (A-Star)
We utilize the A* algorithm for global path planning. It is an optimal search algorithm that finds the shortest path between the robot's current position and its destination on a grid.
*   **Heuristic**: Manhattan Distance (`|x1 - x2| + |y1 - y2|`) is used because robots move in 4 cardinal directions (Grid-based).
*   **Cost Function**: `f(n) = g(n) + h(n)` where `g` is the cost from start and `h` is the heuristic.

### 2. Prioritized Planning (Collision Predictions)
To prevent robots from hitting each other we use a **Prioritized Planning** approach:
*   Robots with higher urgency plan first.
*   The path planner considers the live positions of *other active robots* as **Dynamic Obstacles**.
*   **Look-Ahead**: Before entering a cell, a robot queries the fleet state. If the target cell is occupied, it waits, effectively preventing collisions.

### 3. Dynamic Re-planning (Local Repair)
In real-world warehouses, paths can be blocked by dropped boxes or humans. Orchestrix implements **Real-Time Re-planning**:
1.  **Detection**: Before moving to the next cell, simulated "Lidar" checks for blockages (Static or Dynamic).
2.  **Reaction**: If the path is blocked, the robot halts immediately.
3.  **Re-Calculation**: The robot triggers A* *from its current location* to find a new optimal route.

### 4. Failure Recovery (Smart Retry)
*   **Timeouts**: Tasks stuck `IN_PROGRESS` for > 60s are auto-flagged as `FAILED`.
*   **Auto-Retry**: The WCS monitors for failed tasks and automatically re-queues them (up to 3 times) to be picked up by healthy robots.

---

## 🤖 ROS2 Integration Structure
The `Ros_implementation/` folder contains a complete ROS2 package (`orchestrix_rcs`).

### 📂 File Structure
```text
Ros_implementation/
├── src/
│   └── orchestrix_rcs/
│       ├── package.xml       # Dependencies (rclpy, std_msgs)
│       ├── setup.py          # Build configuration
│       └── orchestrix_rcs/
│           ├── __init__.py
│           └── robot_node.py # MAIN LOGIC
```

### 📡 Topics & Interfaces
The `robot_node.py` facilitates communication:
*   **WCS Interface**: HTTP REST (`GET /tasks`, `POST /register`).
*   **ROS Publishers**:
    *   `/cmd_vel` (`geometry_msgs/Twist`): Velocity commands for the robot base.
    *   `/odom` (`nav_msgs/Odometry`): Robot position updates.
    *   `/robot_status` (`std_msgs/String`): Status reporting.

---

## 💻 Tech Stack

### Backend
*   **Language**: Python 3.12+
*   **Framework**: **FastAPI** (Async).
*   **Server**: Uvicorn.
*   **Design**: Modular (WMS, WCS, Robots, Map).

### Frontend
*   **Library**: **React 18** (Vite).
*   **Styling**: CSS Modules + Glassmorphism aesthetic.
*   **Network**: Axios.
*   **Visualization**: Custom CSS Grid.

---

## ⚙️ How to Run

### Prerequisites
*   Python 3.10+ installed.
*   Node.js & npm installed.

### 1. Installation
```bash
# Backend Deps
pip install fastapi uvicorn pydantic requests

# Frontend Deps
cd frontend
npm install
```

### 2. Run the Platform (Simulated)
Double-click the **`start_orchestrator.bat`** file in the root directory.
*   Starts Backend at `http://localhost:8000`
*   Starts Frontend at `http://localhost:5173`

### 3. Run ROS2 Node (Optional)
To run a robot node that connects to the platform:
Double-click **`start_ros_node.bat`**.

*   **If ROS2 is installed**: It runs the node natively (`ros2 run`).
*   **If no ROS2**: It runs in **MOCK MODE**, simulating topic publications so you can still verify the logic without Linux/ROS.

---

## 🎮 How to Use

1.  **Dispatch Tasks**: 
    *   Use the **OPERATIONS** panel on the right.
    *   Select `Source` (e.g., RECEIVING) and `Destination` (e.g., SHIPPING).
    *   Click **SEND GOAL**. The system will assign an idle robot.

2.  **Manage Fleet**:
    *   Click **+ ADD NEW ROBOT**.
    *   **Priority**: Assign Importance (Low/Medium/High) to new robots.

3.  **Simulate Obstacles**:
    *   **Click Map**: Click any empty grid cell to place an obstacle.
    *   **Watch Behavior**: Robots will pause and re-plan around it.

4.  **Kill Switch (Failure Test)**:
    *   Click the **"💀"** button on a robot card.
    *   The robot will fail, and the system will automatically re-assign its task to another robot.

---
*Built for Advanced Warehouse Automation Studies.*

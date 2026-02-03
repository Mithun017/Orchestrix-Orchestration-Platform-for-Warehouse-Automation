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

---

## 🧠 Algorithms Used

### 1. A* Pathfinding (A-Star)
We utilize the A* algorithm for global path planning. It is an optimal search algorithm that finds the shortest path between the robot's current position and its destination on a grid.
*   **Heuristic**: Manhattan Distance (`|x1 - x2| + |y1 - y2|`).
*   **Cost Function**: `f(n) = g(n) + h(n)` where `g` is the cost from start and `h` is the heuristic.

### 2. Dynamic Re-planning (Local Repair)
In real-world warehouses, paths can be blocked by dropped boxes or humans. Orchestrix implements **Real-Time Re-planning**:
1.  **Detection**: Before moving to the next cell, the robot sensors (simulated) check for blockages.
2.  **Reaction**: If the path is blocked, the robot halts immediately.
3.  **Re-Calculation**: The robot triggers the A* algorithm *from its current location* to find a new optimal route to the destination, treating the blocked cell as a static obstacle.

---

## 💻 Tech Stack

### Backend
*   **Language**: Python 3.12+
*   **Framework**: **FastAPI** (High performance, async support).
*   **Server**: Uvicorn (ASGI).
*   **Architecture**: Modular (WMS, WCS, Robots, Map separated).

### Frontend
*   **Library**: **React 18** (Vite).
*   **Styling**: CSS Modules + Glassmorphism aesthetic.
*   **Network**: Axios (Polled state synchronization).
*   **Visualization**: Custom Grid Rendering System.

---

## ⚙️ How to Run

### Prerequisites
*   Python 3.10+ installed.
*   Node.js & npm installed.

### Setup Steps
1.  **Install Python Dependencies**:
    ```bash
    pip install fastapi uvicorn pydantic requests
    ```
2.  **Install Frontend Dependencies**:
    ```bash
    cd frontend
    npm install
    ```

### Running the Platform
Simply run the included batch script from the root directory:
```bash
./start_orchestrator.bat
```
This script will:
*   Start the Backend API on `http://127.0.0.1:8000`
*   Start the Frontend UI on `http://localhost:5173`

---

## 🎮 How to Use (User Guide)

1.  **Dispatch Tasks**: 
    *   Use the **OPERATIONS** panel on the right.
    *   Select `Source` (e.g., RECEIVING) and `Destination` (e.g., SHIPPING).
    *   Click **SEND GOAL**. The system will assign an idle robot.

2.  **Manage Fleet**:
    *   Click **+ ADD NEW ROBOT** to spawn more agents into the fleet.

3.  **Simulate Obstacles (Dynamic Environment)**:
    *   **Click Map**: Click any empty grid cell to place a "Sudden Obstacle".
    *   **Watch Behavior**: If a robot is moving towards that cell, it will pause and find a new way around.
    
4.  **Environment Actions**:
    *   **Reset Layout**: Restore original map.
    *   **Clear All**: Remove all obstacles.
    *   **Randomize**: Add 5 random obstacles to test robustness.

---
*Built for Advanced Warehouse Automation Studies.*

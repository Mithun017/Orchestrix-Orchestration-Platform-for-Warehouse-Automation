@echo off
echo ==================================================
echo   ORCHESTRIX SIMULATION SUITE (ROS/RCS + VIZ)
echo ==================================================
echo.

:: 1. Launch Robot Control Node (The "Driver") in a new independent window
:: We use a temporary block to handle the complex logic inside 'start'
echo Launching RCS Node...
start "Orchestrix RCS Node" cmd /k "echo Starting Robot Node... & where ros2 >nul 2>nul && (echo [INFO] ROS2 detected. & cd Ros_implementation & call colcon build --packages-select orchestrix_rcs & call install/setup.bat & call ros2 run orchestrix_rcs robot_node) || (echo [INFO] ROS2 missing. Running MOCK MODE. & python Ros_implementation/src/orchestrix_rcs/orchestrix_rcs/robot_node.py)"

:: 2. Launch Visualizer (The "RViz" Interface) in a new independent window
echo Launching Visualizer...
start "Orchestrix Simulation View" cmd /k "echo Starting Visualizer... & python Ros_implementation/visualizer.py"

echo.
echo [INFO] Simulation Environment Launched.
echo [INFO] You should see two new windows:
echo        1. The Robot Logic (Logs)
echo        2. The Map Visualizer (GUI)
echo.
echo Note: Ensure the Backend (start_orchestrator.bat) is running for data!
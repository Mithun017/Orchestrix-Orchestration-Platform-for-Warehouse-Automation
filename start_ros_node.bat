@echo off
echo Starting ORCHESTRIX ROS2 Node Interface...
echo.

:: Check if ROS2 is installed and on the path
where ros2 >nul 2>nul
if %errorlevel%==0 (
    echo [INFO] ROS2 detected in environment.
    cd Ros_implementation
    
    echo Building package...
    call colcon build --packages-select orchestrix_rcs
    
    echo Sourcing environment...
    call install/setup.bat
    
    echo Launching Robot Node...
    call ros2 run orchestrix_rcs robot_node
) else (
    echo [INFO] 'ros2' command not found.
    echo [INFO] Falling back to MOCK MODE (Python direct execution).
    echo [INFO] This simulates the Robot Node without requiring ROS2 installation.
    echo.
    python "Ros_implementation/src/orchestrix_rcs/orchestrix_rcs/robot_node.py"
)

pause

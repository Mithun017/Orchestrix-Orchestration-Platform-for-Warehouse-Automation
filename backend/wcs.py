from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import List, Dict
from .models import Task, Robot, RobotStatus, TaskStatus, TaskUpdate
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("WCS")

router = APIRouter(prefix="/wcs", tags=["WCS"])

# WCS State
wcs_tasks: Dict[str, Task] = {}
robots: Dict[str, Robot] = {}

# --- INTERNAL LOGIC ---

async def receive_task_from_wms(task: Task):
    """Called by WMS to push a new task to WCS"""
    logger.info(f"WCS Received Task: {task.task_id}")
    wcs_tasks[task.task_id] = task
    # Trigger assignment logic
    await try_assign_tasks()

async def try_assign_tasks():
    """
    Core Orchestration Logic: Match Idle Robots to Pending Tasks.
    Priority Queue Logic: Sort by Priority (Desc) then Created Time.
    """
    # Filter pending tasks
    pending_tasks = [t for t in wcs_tasks.values() if t.status == TaskStatus.CREATED]
    # Sort: Higher priority first
    pending_tasks.sort(key=lambda x: x.priority, reverse=True)
    
    # Filter idle robots
    idle_robots = [r for r in robots.values() if r.status == RobotStatus.IDLE]
    
    if not pending_tasks:
        logger.info("No pending tasks.")
        return
    
    if not idle_robots:
        logger.info("No idle robots available.")
        return

    # Assignment Loop
    while pending_tasks and idle_robots:
        task = pending_tasks.pop(0)
        robot = idle_robots.pop(0)
        
        await assign_task_to_robot(task, robot)

async def assign_task_to_robot(task: Task, robot: Robot):
    logger.info(f"Assigning Task {task.task_id} to Robot {robot.robot_id}")
    
    # Update State
    task.status = TaskStatus.ASSIGNED
    task.assigned_robot_id = robot.robot_id
    
    robot.status = RobotStatus.BUSY
    robot.current_task_id = task.task_id
    
    # Notify Robot (Simulated API call)
    from .robots import command_robot_start_task
    await command_robot_start_task(robot.robot_id, task)

    # Notify WMS of update
    from .wms import update_task_status
    await update_task_status(task.task_id, TaskUpdate(status=TaskStatus.ASSIGNED))

# --- API ENDPOINTS ---

@router.get("/tasks", response_model=List[Task])
async def get_wcs_tasks():
    return list(wcs_tasks.values())

@router.get("/robots", response_model=List[Robot])
async def get_robots():
    return list(robots.values())

# Called by Robot to report status
async def report_robot_completion(robot_id: str, task_id: str):
    logger.info(f"Robot {robot_id} completed Task {task_id}")
    
    if task_id in wcs_tasks:
        wcs_tasks[task_id].status = TaskStatus.COMPLETED
        # Notify WMS
        from .wms import update_task_status
        await update_task_status(task_id, TaskUpdate(status=TaskStatus.COMPLETED, progress=100.0))
        
    if robot_id in robots:
        robots[robot_id].status = RobotStatus.IDLE
        robots[robot_id].current_task_id = None
        
    # Trigger next assignment
    await try_assign_tasks()
    
async def report_robot_progress(robot_id: str, task_id: str, progress: float):
    if task_id in wcs_tasks:
        val = wcs_tasks[task_id]
        if val.status != TaskStatus.IN_PROGRESS:
             val.status = TaskStatus.IN_PROGRESS
             from .wms import update_task_status # Notify WMS of start
             await update_task_status(task_id, TaskUpdate(status=TaskStatus.IN_PROGRESS))
             
        val.progress = progress
        # We could notify WMS of progress updates too, maybe every 25%

from fastapi import APIRouter
from .models import Robot, Task, RobotStatus, TaskStatus, TaskUpdate 
from .wcs import robots as wcs_robots 
from .map import map_manager, get_location_coords 
import asyncio
import uuid
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ROBOTS")

router = APIRouter(prefix="/robots", tags=["Robots"])

async def move_to_target(robot_id: str, target_coords: tuple, task_id: str = None):
    """
    Intelligent movement with Re-planning.
    Updates Robot state step-by-step.
    """
    if robot_id not in wcs_robots:
        return False
        
    robot = wcs_robots[robot_id]
    
    # Max retries/steps to prevent infinite loops
    step_limit = 100
    steps_count = 0

    while (robot.x, robot.y) != target_coords:
        if steps_count > step_limit:
            logger.error(f"Robot {robot_id} stuck or loop detected. Aborting.")
            return False
        steps_count += 1

        # 1. Gather Dynamic Obstacles (Other Robots)
        other_robots_pos = []
        for r_id, r in wcs_robots.items():
            if r_id != robot_id and r.status != RobotStatus.IDLE: # Only consider active robots as obstacles? Or all? 
                # Consider ALL robots as obstacles to avoid crashing into idle ones
                other_robots_pos.append((r.x, r.y))

        # 2. Plan Path from Current Location
        current_pos = (robot.x, robot.y)
        # Pass other robots as obstacles
        path = map_manager.find_path(current_pos, target_coords, dynamic_obstacles=other_robots_pos)
        robot.path = path 
        
        if not path or len(path) < 2:
            if path and path[0] == target_coords:
                break
            
            logger.warning(f"Robot {robot_id} blocked by static/dynamic obstacles.")
            # Wait a bit, maybe the other robot moves
            await asyncio.sleep(1)
            # Check timeout/retry logic here if strict
            continue # Retry loop

        # 3. Identify Next Step
        next_pos = path[1]

        # 4. Check Collision (Double Check before moving)
        # Check Static
        if map_manager.is_blocked(next_pos[0], next_pos[1]):
            logger.warning(f"Robot {robot_id} blocked by Static Obstacle. Re-planning...")
            await asyncio.sleep(0.5)
            continue
            
        # Check Dynamic (Is any robot currently at next_pos?)
        collision = False
        for r_id, r in wcs_robots.items():
             if r_id != robot_id and (r.x, r.y) == next_pos:
                 collision = True
                 break
        
        if collision:
             logger.warning(f"Robot {robot_id} blocked by another Robot at {next_pos}. Waiting...")
             await asyncio.sleep(1) # Wait for it to move
             continue

        # 5. Move
        robot.x, robot.y = next_pos
        
        # 6. Simulate Speed
        await asyncio.sleep(0.5) 

        # 6. Report Progress (Mock)
        # can send updates here if needed

    return True

async def execute_complex_task(robot_id: str, task: Task):
    if robot_id not in wcs_robots:
        return

    robot = wcs_robots[robot_id]
    
    try:
        # 1. Go to Source
        source_coords = get_location_coords(task.source)
        logger.info(f"Robot {robot_id} moving to Source {task.source} {source_coords}")
        success = await move_to_target(robot_id, source_coords, task.task_id)
        
        if not success:
             raise Exception(f"Failed to reach source {task.source}")

        # 2. Pick up
        await asyncio.sleep(1)
        
        # 3. Go to Destination
        dest_coords = get_location_coords(task.destination)
        logger.info(f"Robot {robot_id} moving to Dest {task.destination} {dest_coords}")
        success = await move_to_target(robot_id, dest_coords, task.task_id)

        if not success:
             raise Exception(f"Failed to reach destination {task.destination}")
        
        # 4. Complete
        from .wcs import report_robot_completion
        robot.path = []
        await report_robot_completion(robot_id, task.task_id)

    except Exception as e:
        logger.error(f"Robot {robot_id} Task Failed: {e}")
        # Reset Robot to IDLE so it is not stuck
        robot.status = RobotStatus.IDLE
        robot.current_task_id = None
        robot.path = []
        
        from .wms import update_task_status
        await update_task_status(task.task_id, TaskUpdate(status=TaskStatus.FAILED))


@router.post("/register", response_model=Robot)
async def register_robot(priority: int = 1):
    robot_id = f"R-{str(uuid.uuid4())[:4].upper()}"
    wcs_robots[robot_id] = Robot(robot_id=robot_id, x=0, y=0, priority=priority)
    return wcs_robots[robot_id]

async def command_robot_start_task(robot_id: str, task: Task):
    # Run in background
    asyncio.create_task(execute_complex_task(robot_id, task))
    return {"msg": "Command received"}

@router.post("/{robot_id}/fail")
async def fail_robot(robot_id: str):
    if robot_id not in wcs_robots:
        return {"error": "Robot not found"}
    
    robot = wcs_robots[robot_id]
    robot.status = RobotStatus.ERROR
    
    # Fail current task if any
    if robot.current_task_id:
        from .wcs import wcs_tasks # direct import to modify state
        if robot.current_task_id in wcs_tasks:
            wcs_tasks[robot.current_task_id].status = TaskStatus.FAILED
            # The WCS monitor will pick this up and retry it
            
    return {"msg": f"Robot {robot_id} simulated failure."}

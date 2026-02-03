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

        # 1. Plan Path from Current Location
        current_pos = (robot.x, robot.y)
        path = map_manager.find_path(current_pos, target_coords)
        robot.path = path # Visual debug
        
        if not path or len(path) < 2:
            # Check if we are already there (len=1 and path[0] == target)
            if path and path[0] == target_coords:
                break
            
            logger.warning(f"Robot {robot_id} cannot find path from {current_pos} to {target_coords}")
            return False

        # 2. Identify Next Step
        # path[0] is current, path[1] is next
        next_pos = path[1]

        # 3. Check Obstacle (Just before moving)
        if map_manager.is_blocked(next_pos[0], next_pos[1]):
            logger.warning(f"Robot {robot_id} path blocked at {next_pos}. Re-planning...")
            # We just continue the loop -> next iteration will re-call find_path()
            # which will see the obstacle (assuming map_manager is updated)
            # IMPORTANT: map_manager.is_blocked checks the *live* obstacle set.
             
            # Brief pause before re-planning to not spam CPU
            await asyncio.sleep(0.5)
            continue
        
        # 4. Move
        robot.x, robot.y = next_pos
        
        # 5. Simulate Speed
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

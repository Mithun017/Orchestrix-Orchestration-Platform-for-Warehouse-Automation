from fastapi import APIRouter, HTTPException
from typing import List, Dict
import uuid
from .models import Task, TaskCreate, TaskStatus, TaskUpdate
import time

router = APIRouter(prefix="/wms", tags=["WMS"])

# Mock Database for WMS
wms_tasks: Dict[str, Task] = {}

@router.post("/tasks", response_model=Task)
async def create_task(task_in: TaskCreate):
    task_id = f"T-{str(uuid.uuid4())[:8]}"
    new_task = Task(
        task_id=task_id,
        source=task_in.source,
        destination=task_in.destination,
        priority=task_in.priority,
        estimated_time=task_in.estimated_time,
        created_at=time.time(),
        updated_at=time.time()
    )
    wms_tasks[task_id] = new_task
    
    # Notify WCS
    # Import inside function to avoid circular import during initialization
    from .wcs import receive_task_from_wms
    await receive_task_from_wms(new_task)
    
    return new_task

@router.get("/tasks", response_model=List[Task])
async def list_tasks():
    return list(wms_tasks.values())

@router.post("/tasks/{task_id}/status")
async def update_task_status(task_id: str, update: TaskUpdate):
    """
    Called by WCS or Robots to update task status/progress.
    Renamed from update_wms_task_status to match import in wcs.py
    """
    if task_id not in wms_tasks:
        # If task not found (maybe restart cleared memory), just ignore or log
        # For simulation, we can re-create or ignoring is safer than crashing
        return {"msg": "Task not found, ignoring update"}
    
    task = wms_tasks[task_id]
    if update.status:
        task.status = update.status
    if update.progress is not None:
        task.progress = update.progress
        
    return task

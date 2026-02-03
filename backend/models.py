from pydantic import BaseModel
from typing import Optional, List
from enum import Enum
import time

class TaskStatus(str, Enum):
    CREATED = "CREATED"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"

class RobotStatus(str, Enum):
    IDLE = "IDLE"
    BUSY = "BUSY"
    ERROR = "ERROR" # Robot reported error or went offline

class Task(BaseModel):
    task_id: str
    source: str
    destination: str
    priority: int
    estimated_time: int
    status: TaskStatus = TaskStatus.CREATED
    assigned_robot_id: Optional[str] = None
    progress: float = 0.0
    created_at: float = 0.0
    updated_at: float = 0.0
    retry_count: int = 0

class TaskCreate(BaseModel):
    source: str
    destination: str
    priority: int = 5
    estimated_time: int = 10

class TaskUpdate(BaseModel):
    status: Optional[TaskStatus] = None
    progress: Optional[float] = None

class Robot(BaseModel):
    robot_id: str
    status: RobotStatus = RobotStatus.IDLE
    priority: int = 1 
    current_task_id: Optional[str] = None
    x: int = 0
    y: int = 0
    path: List[tuple] = []

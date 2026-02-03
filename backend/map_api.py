from fastapi import APIRouter
from typing import List, Tuple
from pydantic import BaseModel
from .map import map_manager

router = APIRouter(prefix="/map", tags=["Map"])

class ObstacleUpdate(BaseModel):
    x: int
    y: int

@router.get("/obstacles")
async def get_obstacles():
    return map_manager.get_obstacles()

@router.post("/toggle-obstacle")
async def toggle_obstacle(update: ObstacleUpdate):
    is_now_obstacle = map_manager.toggle_obstacle(update.x, update.y)
    return {"x": update.x, "y": update.y, "is_obstacle": is_now_obstacle}

@router.post("/reset")
async def reset_map():
    map_manager.reset_obstacles()
    return {"msg": "Map reset to default"}

@router.post("/clear")
async def clear_map():
    map_manager.clear_obstacles()
    return {"msg": "All obstacles cleared"}

@router.post("/randomize")
async def randomize_map():
    map_manager.generate_random_obstacles()
    return {"msg": "Random obstacles added"}

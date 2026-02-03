from typing import List, Tuple, Dict, Set
import random

# Grid Configuration
GRID_SIZE = 10 

# Initial Static Obstacles
INITIAL_OBSTACLES: List[Tuple[int, int]] = [
    (2, 2), (2, 3), (2, 4), # Shelf A
    (7, 2), (7, 3), (7, 4), # Shelf B
    (4, 7), (5, 7), (6, 7)  # Central Hub
]

# Defined Locations
LOCATIONS: Dict[str, Tuple[int, int]] = {
    "RECEIVING": (0, 0),
    "SHIPPING": (9, 9),
    "STORAGE-A": (1, 3),
    "STORAGE-B": (8, 3),
    "CHARGING": (5, 5),
    "PACKING": (0, 9),
    "PICKING": (9, 0)
}

class MapManager:
    def __init__(self, grid_size: int, initial_obstacles: List[Tuple[int, int]]):
        self.width = grid_size
        self.height = grid_size
        self.initial_obstacles = set(initial_obstacles) # Save for reset
        self.obstacles: Set[Tuple[int, int]] = set(initial_obstacles)

    def add_obstacle(self, x: int, y: int):
        self.obstacles.add((x, y))

    def remove_obstacle(self, x: int, y: int):
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))

    def toggle_obstacle(self, x: int, y: int):
        if (x, y) in self.obstacles:
            self.obstacles.remove((x, y))
            return False 
        else:
            self.obstacles.add((x, y))
            return True 

    def is_blocked(self, x: int, y: int) -> bool:
        return (x, y) in self.obstacles
    
    def get_obstacles(self) -> List[Tuple[int, int]]:
        return list(self.obstacles)
        
    def reset_obstacles(self):
        self.obstacles = self.initial_obstacles.copy()
        
    def clear_obstacles(self):
        self.obstacles = set()

    def generate_random_obstacles(self, count=5):
        added = 0
        attempts = 0
        while added < count and attempts < 100:
            x = random.randint(0, self.width - 1)
            y = random.randint(0, self.height - 1)
            # Don't block locations or (0,0)/(9,9) critical paths roughly
            # Also simple check if it conflicts with existing
            if (x,y) not in self.obstacles and (x,y) not in LOCATIONS.values():
                self.obstacles.add((x, y))
                added += 1
            attempts += 1

    # Pathfinding Methods
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        x, y = node
        candidates = [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1) 
        ]
        results = []
        for nx, ny in candidates:
            if 0 <= nx < self.width and 0 <= ny < self.height:
                if (nx, ny) not in self.obstacles:
                    results.append((nx, ny))
        return results

    def find_path(self, start: Tuple[int, int], goal: Tuple[int, int], dynamic_obstacles: List[Tuple[int, int]] = None) -> List[Tuple[int, int]]:
        if dynamic_obstacles is None:
            dynamic_obstacles = []
            
        all_obstacles = self.obstacles.union(set(dynamic_obstacles))

        if start in all_obstacles or goal in all_obstacles:
            # If start is blocked by a dynamic obstacle (e.g. another robot), we might be stuck.
            # But usually we are AT start. So usually start shouldn't be considered "blocked" for the purpose of *being there*, but we can't move *back* to it?
            # Actually, standard A* fails if start/goal is blocked.
            # If goal is occupied by another robot, we can't path there. return empty.
            if goal in all_obstacles:
                 return []
        
        if start == goal:
            return [start]
            
        frontier = [(0, start)]
        came_from = {start: None}
        cost_so_far = {start: 0}
        
        while frontier:
            frontier.sort(key=lambda x: x[0])
            current_priority, current = frontier.pop(0)

            if current == goal:
                break

            # Custom get_neighbors that checks dynamic obstacles
            neighbors = []
            x, y = current
            candidates = [(x+1, y), (x-1, y), (x, y+1), (x, y-1)]
            for nx, ny in candidates:
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in all_obstacles:
                        neighbors.append((nx, ny))

            for next_node in neighbors:
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    frontier.append((priority, next_node))
                    came_from[next_node] = current
        
        if goal not in came_from:
            return [] 

        path = []
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

# Global Instance
map_manager = MapManager(GRID_SIZE, INITIAL_OBSTACLES)
pf = map_manager 

def get_location_coords(name: str) -> Tuple[int, int]:
    return LOCATIONS.get(name.upper(), (0, 0))

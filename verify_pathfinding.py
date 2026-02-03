from backend.map import pf

def test_pathfinding():
    print("Testing Pathfinding...")
    
    # Test 1: Simple Path
    start = (0, 0)
    goal = (0, 2)
    path = pf.find_path(start, goal)
    print(f"Path (0,0)->(0,2): {path}")
    assert path == [(0,0), (0,1), (0,2)]
    
    # Test 2: Obstacle Avoidance
    # Obstacle at (2,2)
    start = (1, 2)
    goal = (3, 2)
    path = pf.find_path(start, goal)
    print(f"Path (1,2)->(3,2) [Obstacle at 2,2]: {path}")
    
    # Path should go around, e.g., (1,2)->(1,1)->(2,1)->(3,1)->(3,2) or similar
    assert (2,2) not in path
    assert len(path) > 0
    assert path[-1] == goal
    
    print("SUCCESS: Pathfinding verified.")

if __name__ == "__main__":
    test_pathfinding()

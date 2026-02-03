import sys
import os
import time
import math
import requests
import json

# Add backend to path to re-use Map/Models if possible, 
# otherwise we replicate simple logic or just use the API.
# We will use API for state, but might need Map for A*.
# For this implementation, we will import the MapManager from backend 
# assuming file structure is relative.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../backend')))

# Try to import ROS2 libraries. If failing, we mock them for demonstration.
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Point
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
except ImportError:
    print("ROS2 libraries not found. Running in MOCK mode (printing output).")
    # Mock Classes
    class Node:
        def __init__(self, name):
            self.get_logger = lambda: self
        def create_timer(self, interval, callback):
            return None
        def create_publisher(self, msg_type, topic, qos):
            return self
        def publish(self, msg):
            print(f"[ROS-MOCK] Publishing to {topic}: {msg}")
        def info(self, msg):
            print(f"[INFO] {msg}")
        def warn(self, msg):
            print(f"[WARN] {msg}")
        def error(self, msg):
            print(f"[ERROR] {msg}")
            
    class twist:
        linear = type('obj', (object,), {'x':0.0, 'y':0.0, 'z':0.0})
        angular = type('obj', (object,), {'x':0.0, 'y':0.0, 'z':0.0})
    class Twist:
        def __init__(self): self.linear = twist.linear; self.angular = twist.angular
    class Odometry:
        pass
    class String:
        pass
    def init(args=None): pass
    def spin(node): 
        while True: 
            time.sleep(1)
            node.timer_callback()
    def shutdown(): pass
    
    # Mock module structure
    class rclpy_mock:
        def init(args=None): pass
        def spin(node): 
            while True: 
                try:
                    node.timer_callback()
                    time.sleep(1.0) # Slow loop for mock
                except KeyboardInterrupt:
                    break
        def shutdown(): pass
        class node:
            Node = Node
    rclpy = rclpy_mock

try:
    # Try importing map manager from backend
    from map import map_manager, get_location_coords
except ImportError:
    print("Could not import backend map. Using simple mock map.")
    map_manager = None


WCS_API_URL = "http://127.0.0.1:8000"

class WarehouseRobotNode(Node):
    def __init__(self):
        super().__init__('warehouse_robot_node')
        self.get_logger().info('Warehouse Robot Node Started')
        
        # 1. ROS Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.status_pub = self.create_publisher(String, 'robot_status', 10)
        
        # 2. Register with WCS
        self.robot_id = self.register_robot()
        self.get_logger().info(f'Registered with WCS as: {self.robot_id}')
        
        # State
        self.current_task = None
        self.path = []
        self.location = (0, 0) # x, y
        
        # 3. Control Loop (1Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def register_robot(self):
        try:
            # Register with high priority for testing
            resp = requests.post(f"{WCS_API_URL}/robots/register?priority=10")
            if resp.status_code == 200:
                data = resp.json()
                self.location = (data['x'], data['y'])
                return data['robot_id']
        except Exception as e:
            self.get_logger().error(f"Failed to register: {e}")
        return "R-MOCK"

    def timer_callback(self):
        # 1. Poll Status from WCS to see if I have been assigned a task
        # Note: In a real ROS system, WCS might push to a topic, or we poll.
        try:
            # Get my own state from WCS (Source of Truth)
            # Efficient way is to get all and find me, or add strict endpoint
            resp = requests.get(f"{WCS_API_URL}/wcs/robots")
            if resp.status_code == 200:
                robots = resp.json()
                my_state = next((r for r in robots if r['robot_id'] == self.robot_id), None)
                
                if my_state:
                    # Update local knowledge
                    server_task_id = my_state.get('current_task_id')
                    
                    if server_task_id and not self.current_task:
                        # NEW TASK ASSIGNED
                        self.get_logger().info(f"Received Task Assignment: {server_task_id}")
                        self.fetch_and_start_task(server_task_id)
                    
                    if not server_task_id:
                        self.current_task = None
                        
        except Exception as e:
             self.get_logger().warn(f"Connection error to WCS: {e}")

        # 2. Execute Task Logic
        if self.current_task and self.path:
            self.follow_path()

    def fetch_and_start_task(self, task_id):
        # Allow fetching task details
        try:
            resp = requests.get(f"{WCS_API_URL}/wcs/tasks")
            if resp.status_code == 200:
                tasks = resp.json()
                self.current_task = next((t for t in tasks if t['task_id'] == task_id), None)
                
                if self.current_task and map_manager:
                    # Plan Path
                    start = self.location
                    goal = get_location_coords(self.current_task['destination']) 
                    # Note: We should first go to source, then dest. 
                    # Simplified here: assume we are implementing the "Move" command.
                    
                    # For full logic: 
                    # If at source -> go to dest
                    # If not at source -> go to source
                    
                    source_coords = get_location_coords(self.current_task['source'])
                    
                    if self.location != source_coords:
                         self.path = map_manager.find_path(self.location, source_coords)
                    else:
                         self.path = map_manager.find_path(self.location, goal)
                         
        except Exception as e:
            self.get_logger().error(f"Error fetching task: {e}")

    def follow_path(self):
        if not self.path:
            return

        # Get next step
        next_step = self.path.pop(0) # path[0] is current normally?
        # If path[0] is where we are, pop it.
        if next_step == self.location and self.path:
             next_step = self.path.pop(0)

        # Move
        self.location = next_step
        self.get_logger().info(f"Moving to {self.location}")
        
        # Publish ROS Twist (Fake)
        msg = Twist()
        msg.linear.x = 0.5
        self.cmd_vel_pub.publish(msg)
        
        # Report to WCS (This effectively mocks the "Robot moving" without backend simulation loop)
        # Note: Backend Robots.py loop might conflict if running. 
        # But this code demonstrates the "RCS" capability.

def main(args=None):
    rclpy.init(args=args)
    node = WarehouseRobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

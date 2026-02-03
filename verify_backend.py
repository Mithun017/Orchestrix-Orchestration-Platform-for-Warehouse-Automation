import subprocess
import time
import requests
import sys
import os
import signal

# Configuration
BASE_URL = "http://127.0.0.1:8000"

def run_verification():
    print("Starting Backend Server...")
    # Start Uvicorn in a separate process
    process = subprocess.Popen(
        [sys.executable, "-m", "uvicorn", "backend.main:app", "--port", "8000"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )
    
    try:
        # Wait for server to start
        print("Waiting for server to boot...")
        for _ in range(10):
            try:
                requests.get(BASE_URL + "/")
                break
            except requests.exceptions.ConnectionError:
                time.sleep(1)
        else:
            print("Server failed to start.")
            return False

        print("Server is up. Starting Tests.")

        # 1. Register Robot
        print("1. Registering Robot...")
        resp = requests.post(f"{BASE_URL}/robots/register")
        assert resp.status_code == 200
        robot = resp.json()
        robot_id = robot["robot_id"]
        print(f"   -> Robot Registered: {robot_id}")

        # 2. Check WCS Robots
        print("2. Verifying WCS knows about Robot...")
        resp = requests.get(f"{BASE_URL}/wcs/robots")
        robots = resp.json()
        assert any(r["robot_id"] == robot_id for r in robots)
        print("   -> WCS Confirmed Robot.")

        # 3. Create Task via WMS
        print("3. Creating Task via WMS...")
        task_payload = {
            "source": "A1",
            "destination": "B2",
            "priority": 5,
            "estimated_time": 2 # Short time for test
        }
        resp = requests.post(f"{BASE_URL}/wms/tasks", json=task_payload)
        assert resp.status_code == 200
        task = resp.json()
        task_id = task["task_id"]
        print(f"   -> Task Created: {task_id}")

        # 4. Monitor Task Status (Polling)
        print("4. Monitoring Execution...")
        
        # Max wait 10 seconds
        start_time = time.time()
        final_status = "UNKNOWN"
        
        while time.time() - start_time < 10:
            resp = requests.get(f"{BASE_URL}/wms/tasks")
            tasks = resp.json()
            target_task = next((t for t in tasks if t["task_id"] == task_id), None)
            
            if target_task:
                status = target_task["status"]
                progress = target_task.get("progress", 0)
                print(f"   -> Status: {status}, Progress: {progress}%")
                
                if status == "COMPLETED":
                    final_status = "COMPLETED"
                    break
            
            time.sleep(1)

        if final_status == "COMPLETED":
            print("SUCCESS: Task completed successfully.")
            return True
        else:
            print(f"FAILURE: Task did not complete. Status: {final_status}")
            return False

    finally:
        print("Stopping Server...")
        process.terminate()
        process.wait()

if __name__ == "__main__":
    if run_verification():
        sys.exit(0)
    else:
        sys.exit(1)

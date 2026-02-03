import sys
import os

# Add current dir to path
sys.path.append(os.getcwd())

try:
    print("Attempting to import backend modules...")
    from backend import main
    from backend import wms
    from backend import wcs
    from backend import robots
    print("SUCCESS: All backend modules imported correctly.")
except ImportError as e:
    print(f"FAILURE: ImportError detected: {e}")
except Exception as e:
    print(f"FAILURE: Unknown error: {e}")

@echo off
echo Starting ORCHESTRIX System...

:: Start Backend
start cmd /k "echo Starting Backend... & python -m uvicorn backend.main:app --reload --port 8000"

:: Wait a moment
timeout /t 3

:: Start Frontend
cd frontend
start cmd /k "echo Starting Frontend... & npm run dev"

:: Wait for frontend to initialize and open in browser
timeout /t 3
start http://localhost:5173

echo System Started. Access frontend at http://localhost:5173
pause

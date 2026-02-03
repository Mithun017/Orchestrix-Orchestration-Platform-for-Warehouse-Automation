import React from 'react';

const GRID_SIZE = 10;
const CELL_SIZE = 50;
const GAP = 6;

// Static locations (re-used)
const LOCATIONS = {
    "0,0": { name: "RECEIVING", color: "#10b981" },
    "9,9": { name: "SHIPPING", color: "#3b82f6" },
    "1,3": { name: "STORAGE-A", color: "#f59e0b" },
    "8,3": { name: "STORAGE-B", color: "#f59e0b" },
    "5,5": { name: "CHARGING", color: "#6366f1" }
};

const WarehouseMap = ({ robots, obstacles = [], onCellClick }) => {

    const checkObstacle = (x, y) => {
        return obstacles.some(obs => obs[0] === x && obs[1] === y);
    };

    // 1. Render Static Grid Layer
    const renderGrid = () => {
        const grid = [];
        for (let y = 0; y < GRID_SIZE; y++) {
            for (let x = 0; x < GRID_SIZE; x++) {
                const coordKey = `${x},${y}`;
                const isObstacle = checkObstacle(x, y);
                const location = LOCATIONS[coordKey];

                let cellStyle = {
                    width: CELL_SIZE,
                    height: CELL_SIZE,
                    position: 'absolute',
                    left: x * (CELL_SIZE + GAP),
                    top: y * (CELL_SIZE + GAP),
                    background: 'rgba(30, 41, 59, 0.5)',
                    borderRadius: 8,
                    border: '1px solid transparent',
                    cursor: 'pointer',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center'
                };

                let content = null;

                if (isObstacle) {
                    cellStyle.backgroundColor = "#334155";
                    cellStyle.border = "1px solid #475569";
                } else if (location) {
                    cellStyle.backgroundColor = `${location.color}20`;
                    cellStyle.border = `1px solid ${location.color}50`;
                    content = <div style={{ fontSize: '0.7rem', color: location.color, fontWeight: 'bold' }}>{location.name[0]}</div>;
                }

                grid.push(
                    <div
                        key={coordKey}
                        style={cellStyle}
                        className="grid-cell"
                        onClick={() => onCellClick(x, y)}
                    >
                        {content}
                    </div>
                );
            }
        }
        return grid;
    };

    // 2. Render Robot Overlay Layer
    const renderRobots = () => {
        return robots.map(robot => (
            <div
                key={robot.robot_id}
                className="robot-marker"
                style={{
                    width: 40,
                    height: 40,
                    position: 'absolute',
                    // Calculate position based on grid coordinates
                    transform: `translate(${robot.x * (CELL_SIZE + GAP) + 5}px, ${robot.y * (CELL_SIZE + GAP) + 5}px)`,
                    backgroundColor: robot.status === "BUSY" ? "#10b981" : "#94a3b8",
                    boxShadow: `0 0 15px ${robot.status === "BUSY" ? "#10b981" : "#94a3b8"}`,
                    borderRadius: '50%',
                    display: 'flex',
                    alignItems: 'center',
                    justifyContent: 'center',
                    color: 'white',
                    fontWeight: 'bold',
                    fontSize: '0.8rem',
                    zIndex: 20,
                    transition: 'transform 0.5s linear, background-color 0.3s' // Smooth movement!
                }}
            >
                R
            </div>
        ));
    };

    const totalSize = GRID_SIZE * (CELL_SIZE + GAP) - GAP;

    return (
        <div className="warehouse-map-container" style={{ width: totalSize + 48, height: totalSize + 48, position: 'relative' }}>
            <div style={{ position: 'relative', width: totalSize, height: totalSize }}>
                {renderGrid()}
                {renderRobots()}
            </div>
            <style>{`
        .warehouse-map-container {
          background: #0f172a;
          padding: 1.5rem;
          border-radius: 16px;
          border: 1px solid var(--border);
          display: inline-block;
          box-shadow: 0 20px 25px -5px rgba(0, 0, 0, 0.1), 0 10px 10px -5px rgba(0, 0, 0, 0.04);
        }
        .grid-cell:hover {
            border-color: var(--primary) !important;
        }
      `}</style>
        </div>
    );
};

export default WarehouseMap;

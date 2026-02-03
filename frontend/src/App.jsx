import { useState, useEffect } from 'react'
import axios from 'axios'
import WarehouseMap from './components/WarehouseMap'

const API_URL = "http://127.0.0.1:8000"

function App() {
    const [tasks, setTasks] = useState([])
    const [robots, setRobots] = useState([])
    const [obstacles, setObstacles] = useState([])

    // New Task Form
    const [source, setSource] = useState('RECEIVING')
    const [dest, setDest] = useState('SHIPPING')

    // Robot Form
    const [robotPriority, setRobotPriority] = useState(1)

    // Fetch Data Loop
    const fetchData = async () => {
        try {
            const [tasksRes, robotsRes, obsRes] = await Promise.all([
                axios.get(`${API_URL}/wms/tasks`),
                axios.get(`${API_URL}/wcs/robots`),
                axios.get(`${API_URL}/map/obstacles`)
            ])
            setTasks(tasksRes.data.reverse())
            setRobots(robotsRes.data)
            setObstacles(obsRes.data)
        } catch (error) {
            console.error("Fetch Error:", error)
        }
    }

    useEffect(() => {
        fetchData()
        const interval = setInterval(fetchData, 500)
        return () => clearInterval(interval)
    }, [])

    const handleCreateTask = async (e) => {
        e.preventDefault()
        try {
            await axios.post(`${API_URL}/wms/tasks`, {
                source,
                destination: dest,
                priority: 5,
                estimated_time: 10
            })
        } catch (error) {
            console.error(error)
            alert("Failed to create task: " + (error.response?.data?.detail || error.message));
        }
    }

    const handleRegisterRobot = async () => {
        try {
            await axios.post(`${API_URL}/robots/register?priority=${robotPriority}`)
        } catch (e) {
            alert("Failed to register robot: " + e.message)
        }
    }

    const handleToggleObstacle = async (x, y) => {
        try {
            await axios.post(`${API_URL}/map/toggle-obstacle`, { x, y })
            fetchData();
        } catch (e) {
            console.error(e)
        }
    }

    // Environment Controls
    const handleMapAction = async (action) => {
        try {
            await axios.post(`${API_URL}/map/${action}`)
            fetchData();
        } catch (e) {
            console.error(e)
        }
    }

    return (
        <div className="app-container">
            <header style={{ marginBottom: '1.5rem', textAlign: 'center' }}>
                <h1>ORCHESTRIX</h1>
                <p style={{ color: 'var(--text-muted)' }}>Real-Time Orchestration & Simulation</p>
            </header>

            {/* MAIN CONTENT GRID: MAP vs SIDEBAR */}
            <div style={{ display: 'grid', gridTemplateColumns: 'min-content 1fr', gap: '2rem', alignItems: 'start', justifyContent: 'center' }}>

                {/* LEFT: MAP */}
                <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
                    <div className="card" style={{ padding: '0.5rem 1rem', marginBottom: '1rem', border: '1px solid var(--primary)', color: 'var(--primary)' }}>
                        <strong>LIVE FLOOR MAP</strong>
                    </div>
                    <WarehouseMap robots={robots} obstacles={obstacles} onCellClick={handleToggleObstacle} />
                    <div style={{ marginTop: '1rem', display: 'flex', justifyContent: 'center', gap: '1.5rem', fontSize: '0.9rem' }}>
                        <span style={{ display: 'flex', alignItems: 'center', gap: '8px' }}><div style={{ width: 10, height: 10, background: '#10b981', borderRadius: '50%' }}></div> Recv</span>
                        <span style={{ display: 'flex', alignItems: 'center', gap: '8px' }}><div style={{ width: 10, height: 10, background: '#3b82f6', borderRadius: '50%' }}></div> Ship</span>
                        <span style={{ display: 'flex', alignItems: 'center', gap: '8px' }}><div style={{ width: 10, height: 10, background: '#334155', borderRadius: '2px' }}></div> Obstacle</span>
                    </div>
                </div>

                {/* RIGHT: UNIFIED CONTROL CENTER */}
                <div className="card" style={{ height: '100%', display: 'flex', flexDirection: 'column', gap: '2rem' }}>

                    {/* 1. OPERATIONS (Task & Fleet) */}
                    <div>
                        <h2 style={{ borderBottom: '1px solid var(--border)', paddingBottom: '0.5rem', marginBottom: '1rem' }}>
                            OPERATIONS
                        </h2>

                        {/* Task Form */}
                        <div style={{ marginBottom: '1.5rem' }}>
                            <h3 style={{ fontSize: '0.9rem', color: 'var(--text-muted)', marginBottom: '0.5rem' }}>DISPATCH TASK</h3>
                            <form onSubmit={handleCreateTask} style={{ display: 'flex', flexDirection: 'column', gap: '1rem' }}>
                                <div style={{ display: 'flex', gap: '1rem' }}>
                                    <div style={{ flex: 1 }}>
                                        <label style={{ fontSize: '0.8rem' }}>From</label>
                                        <select style={{ width: '100%', padding: '0.5rem' }} value={source} onChange={e => setSource(e.target.value)}>
                                            <option value="RECEIVING">RECEIVING</option>
                                            <option value="STORAGE-A">STORAGE-A</option>
                                            <option value="STORAGE-B">STORAGE-B</option>
                                            <option value="CHARGING">CHARGING</option>
                                        </select>
                                    </div>
                                    <div style={{ flex: 1 }}>
                                        <label style={{ fontSize: '0.8rem' }}>To</label>
                                        <select style={{ width: '100%', padding: '0.5rem' }} value={dest} onChange={e => setDest(e.target.value)}>
                                            <option value="SHIPPING">SHIPPING</option>
                                            <option value="PACKING">PACKING</option>
                                            <option value="PICKING">PICKING</option>
                                            <option value="STORAGE-A">STORAGE-A</option>
                                        </select>
                                    </div>
                                </div>
                                <button type="submit" style={{ width: '100%' }}>SEND GOAL</button>
                            </form>
                        </div>

                        {/* Fleet Control */}
                        <div>
                            <div style={{ marginBottom: '0.5rem', display: 'flex', gap: '0.5rem', alignItems: 'center' }}>
                                <label style={{ fontSize: '0.8rem' }}>Priority:</label>
                                <select
                                    value={robotPriority}
                                    onChange={(e) => setRobotPriority(Number(e.target.value))}
                                    style={{ padding: '0.2rem', flex: 1 }}
                                >
                                    <option value={1}>Low (1)</option>
                                    <option value={5}>Medium (5)</option>
                                    <option value={10}>High (10)</option>
                                </select>
                            </div>
                            <button onClick={handleRegisterRobot} style={{ width: '100%', background: 'transparent', border: '1px solid var(--primary)', color: 'var(--primary)' }}>
                                + ADD NEW ROBOT
                            </button>
                        </div>
                    </div>

                    {/* 2. ENVIRONMENT */}
                    <div>
                        <h2 style={{ borderBottom: '1px solid var(--border)', paddingBottom: '0.5rem', marginBottom: '1rem' }}>
                            ENVIRONMENT
                        </h2>

                        <div style={{ background: 'rgba(51, 65, 85, 0.3)', padding: '1rem', borderRadius: '8px', marginBottom: '1rem' }}>
                            <div style={{ fontSize: '0.9rem', marginBottom: '0.5rem' }}><strong>Edit Mode</strong></div>
                            <p style={{ fontSize: '0.8rem', color: 'var(--text-muted)' }}>
                                Click any map cell to toggle <br />
                                <span style={{ color: 'var(--warning)' }}>■ Obstacles</span>
                            </p>
                        </div>

                        <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '0.8rem' }}>
                            <button onClick={() => handleMapAction('reset')} style={{ fontSize: '0.8rem', background: '#475569' }}>
                                ↺ Reset Map
                            </button>
                            <button onClick={() => handleMapAction('clear')} style={{ fontSize: '0.8rem', background: '#ef4444' }}>
                                ✕ Clear All
                            </button>
                            <button onClick={() => handleMapAction('randomize')} style={{ gridColumn: '1/-1', fontSize: '0.8rem', background: '#d97706' }}>
                                ⚡ Randomize Obstacles
                            </button>
                        </div>
                    </div>

                    {/* 3. STATUS */}
                    <div style={{ marginTop: 'auto', paddingTop: '1rem', borderTop: '1px solid var(--border)' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem' }}>
                            <span>Active Robots:</span>
                            <strong>{robots.length}</strong>
                        </div>
                        <div style={{ display: 'flex', justifyContent: 'space-between', fontSize: '0.9rem', marginTop: '0.5rem' }}>
                            <span>Obstacles:</span>
                            <strong>{obstacles.length}</strong>
                        </div>
                    </div>

                </div>

            </div>

            {/* SECTION 3: BOTTOM QUEUES */}
            <section style={{ display: 'grid', gridTemplateColumns: '2fr 1fr', gap: '2rem', marginTop: '3rem' }}>
                {/* TASK QUEUE */}
                <div className="card">
                    <h2>TASK QUEUE</h2>
                    <div className="task-list" style={{ maxHeight: '400px', overflowY: 'auto' }}>
                        {tasks.map(task => (
                            <div key={task.task_id} className={`task-item status-${task.status}`}>
                                <div style={{ flex: 1 }}>
                                    <div style={{ display: 'flex', gap: '1rem', alignItems: 'center' }}>
                                        <strong>{task.source} → {task.destination}</strong>
                                        <span className={`badge status-${task.status}`}>{task.status}</span>
                                    </div>
                                    {task.status === 'IN_PROGRESS' && (
                                        <div style={{ marginTop: '0.5rem', width: '100%', height: '6px', background: '#334155', borderRadius: 3 }}>
                                            <div style={{ width: `${task.progress}%`, height: '100%', background: '#10b981', borderRadius: 3, transition: 'width 0.5s' }}></div>
                                        </div>
                                    )}
                                </div>
                                <div style={{ fontSize: '0.9rem', color: 'var(--text-muted)' }}>
                                    {task.task_id}
                                </div>
                            </div>
                        ))}
                        {tasks.length === 0 && <p style={{ opacity: 0.5 }}>No tasks.</p>}
                    </div>
                </div>

                {/* ROBOT FLEET STATUS */}
                <div className="card">
                    <h2>FLEET STATUS</h2>
                    <div className="robots-grid" style={{ gridTemplateColumns: '1fr' }}>
                        {robots.map(robot => (
                            <div key={robot.robot_id} style={{ padding: '1rem', background: 'rgba(0,0,0,0.2)', borderRadius: '8px', display: 'flex', justifyContent: 'space-between', alignItems: 'center', borderLeft: `4px solid ${robot.status === 'BUSY' ? 'var(--success)' : 'var(--text-muted)'}` }}>
                                <div>
                                    <div style={{ fontWeight: 'bold' }}>{robot.robot_id}</div>
                                    <div style={{ fontSize: '0.8rem', opacity: 0.7 }}>Pos: ({robot.x}, {robot.y})</div>
                                </div>
                                <span className={`badge robot-${robot.status}`}>{robot.status}</span>
                            </div>
                        ))}
                        {robots.length === 0 && <p style={{ opacity: 0.5 }}>No robots online.</p>}
                    </div>
                </div>
            </section>
        </div>
    )
}

export default App

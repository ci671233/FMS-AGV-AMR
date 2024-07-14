import React, { useState, useEffect } from 'react';
import axios from 'axios';

function MonitorPage() {
    const [mapImage, setMapImage] = useState(null);
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState(null);
    const [logs, setLogs] = useState([]);

    useEffect(() => {
        const fetchMapData = async () => {
            try {
                const imageResponse = await axios.get('http://localhost:5557/maps/selected');
                setMapImage(imageResponse.data);

                const robotsResponse = await axios.get('http://localhost:5559/robots/positions');
                setRobots(robotsResponse.data);

                const logsResponse = await axios.get('http://localhost:5561/logs/latest');
                setLogs(logsResponse.data);
            } catch (error) {
                console.error('Error fetching map data:', error);
            }
        };

        fetchMapData();
    }, []);

    const handleRobotClick = (robot) => {
        setSelectedRobot(robot);
    };

    return (
        <div style={{ display: 'flex', flexDirection: 'column' }}>
            <div style={{ flex: 1, display: 'flex' }}>
                <div>
                    {mapImage && (
                        <div>
                            <h2>Map</h2>
                            <img
                                src={`data:image/pgm;base64,${mapImage}`}
                                alt="Map"
                                style={{ position: 'relative', width: '500px', height: '500px' }}
                            />
                            {robots.map(robot => (
                                <div
                                    key={robot.id}
                                    onClick={() => handleRobotClick(robot)}
                                    style={{
                                        position: 'absolute',
                                        left: `${robot.x}px`,
                                        top: `${robot.y}px`,
                                        width: '10px',
                                        height: '10px',
                                        backgroundColor: 'red',
                                        borderRadius: '50%'
                                    }}
                                />
                            ))}
                        </div>
                    )}
                </div>
                <div>
                    {selectedRobot && (
                        <div>
                            <h2>Robot Details</h2>
                            <p>Name: {selectedRobot.name}</p>
                            <p>Model: {selectedRobot.model}</p>
                            <p>Battery: {selectedRobot.battery}%</p>
                            <p>Position: {selectedRobot.x}, {selectedRobot.y}</p>
                            <p>Status: {selectedRobot.status}</p>
                        </div>
                    )}
                </div>
            </div>
            <div style={{ flex: 1 }}>
                <h2>Logs</h2>
                <ul>
                    {logs.map(log => (
                        <li key={log.id}>
                            {log.timestamp}: {log.message}
                        </li>
                    ))}
                </ul>
            </div>
        </div>
    );
}

export default MonitorPage;


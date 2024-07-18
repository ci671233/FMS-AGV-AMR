import React, { useState, useEffect } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MonitoringPage() {
  const [mapImage, setMapImage] = useState('');
  const [robotPositions, setRobotPositions] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [logs, setLogs] = useState([]);
  const [error, setError] = useState(null);

  useEffect(() => {
    const fetchMonitoringData = async () => {
      try {
        const token = localStorage.getItem('token');
        if (!token) {
          setError('No token found. Please log in.');
          return;
        }

        console.log('Token:', token);

        const response = await axios.get('http://localhost:5558/monitoring', {
          headers: {
            Authorization: `Bearer ${token}`
          }
        });

        console.log('Monitoring Data Response:', response.data);

        setMapImage(response.data.mapImage);
        setRobotPositions(response.data.robot_position);
        setLogs(response.data.log_top);
      } catch (error) {
        console.error('Error fetching monitoring data:', error);
        setError('Failed to fetch monitoring data.');
      }
    };

    fetchMonitoringData();
  }, []);

  const handleRobotClick = async (robotId) => {
    try {
      const token = localStorage.getItem('token');
      if (!token) {
        setError('No token found. Please log in.');
        return;
      }

      console.log('Token:', token);

      const response = await axios.get(`http://localhost:5556/robot/${robotId}`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });

      console.log('Robot Details Response:', response.data);

      setSelectedRobot(response.data);
    } catch (error) {
      console.error('Error fetching robot details:', error);
      setError('Failed to fetch robot details.');
    }
  };

  return (
    <div style={{ display: 'flex' }}>
      <div style={{ flex: 1 }}>
        <header>
            <UserInfo />
            <LogoutButton />
        </header>
        <Navbar />
        <h2>Monitoring</h2>
        {error && <p style={{ color: 'red' }}>{error}</p>}
        {mapImage ? (
          <div style={{ position: 'relative', width: '600px', height: '400px' }}>
            <img src={mapImage.mapImage} alt="Selected Map" style={{ width: '100%', height: '100%' }} />
            {robotPositions.map((robot) => (
              <div
                key={robot.id}
                onClick={() => handleRobotClick(robot.id)}
                style={{
                  position: 'absolute',
                  left: `${robot.x}px`,
                  top: `${robot.y}px`,
                  width: '20px',
                  height: '20px',
                  backgroundColor: 'red',
                  borderRadius: '50%',
                  cursor: 'pointer'
                }}
              />
            ))}
          </div>
        ) : (
          <p>Loading map...</p>
        )}
      </div>
      <div style={{ flex: 1, paddingLeft: '20px' }}>
        <h3>Robot Details</h3>
        {selectedRobot ? (
          <div>
            <p>ID: {selectedRobot.id}</p>
            <p>Name: {selectedRobot.name}</p>
            <p>Status: {selectedRobot.status}</p>
            <p>Battery: {selectedRobot.battery}%</p>
            <p>Position: ({selectedRobot.x}, {selectedRobot.y})</p>
            <p>Model: {selectedRobot.model}</p>
          </div>
        ) : (
          <p>Click on the robot to see details</p>
        )}
      </div>
      <div style={{ flex: 1, paddingLeft: '20px' }}>
        <h3>Logs</h3>
        <ul>
          {logs.map((log) => (
            <li key={log.id}>{log.message} - {log.timestamp}</li>
          ))}
        </ul>
      </div>
    </div>
  );
}

export default MonitoringPage;




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

        const response = await axios.get('http://localhost:5558/monitoring', {
          headers: {
            Authorization: `Bearer ${token}`
          }
        });

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

      const response = await axios.get(`http://localhost:5556/robot/${robotId}`, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });

      setSelectedRobot(response.data);
    } catch (error) {
      console.error('Error fetching robot details:', error);
      setError('Failed to fetch robot details.');
    }
  };

  return (
    <div style={{ fontFamily: 'Arial, sans-serif', margin: '20px', display: 'flex', flexDirection: 'column' }}>
      <header style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '20px' }}>
        <UserInfo />
        <LogoutButton />
      </header>
      <div style={{ display: 'flex' }}>
        <Navbar />
      </div>
      <h2 style={{ textAlign: 'center', margin: '20px 0' }}>Monitoring</h2>
      {error && <p style={{ color: 'red', textAlign: 'center' }}>{error}</p>}
      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
        <div style={{ flex: 2, marginRight: '20px' }}>
          {mapImage ? (
            <div style={{ position: 'relative', width: '100%', paddingBottom: '56.25%', margin: '0 auto' }}>
              <img src={mapImage.mapImage} alt="Selected Map" style={{ position: 'absolute', top: 0, left: 0, width: '100%', height: '100%' }} />
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
            <p style={{ textAlign: 'center' }}>Loading map...</p>
          )}
        </div>
        <div style={{ flex: 1, marginRight: '20px' }}>
          <h3 style={{ textAlign: 'center' }}>Robot Details</h3>
          {selectedRobot ? (
            <div style={{ textAlign: 'center' }}>
              <p>ID: {selectedRobot.id}</p>
              <p>Name: {selectedRobot.name}</p>
              <p>Status: {selectedRobot.status}</p>
              <p>Battery: {selectedRobot.battery}%</p>
              <p>Position: ({selectedRobot.x}, {selectedRobot.y})</p>
              <p>Model: {selectedRobot.model}</p>
            </div>
          ) : (
            <p style={{ textAlign: 'center' }}>Click on the robot to see details</p>
          )}
        </div>
      </div>
      <div style={{ marginTop: '20px' }}>
        <h3 style={{ textAlign: 'center' }}>Logs</h3>
        <ul style={{ listStyleType: 'none', padding: '0', textAlign: 'center' }}>
          {logs.map((log) => (
            <li key={log.id} style={{ margin: '10px 0' }}>{log.message} - {log.timestamp}</li>
          ))}
        </ul>
      </div>
    </div>
  );
}

export default MonitoringPage;

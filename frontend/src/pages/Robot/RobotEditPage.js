import React, { useState, useEffect } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function RobotEditPage() {
  const [robots, setRobots] = useState([]);
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [name, setName] = useState('');
  const [ip, setIp] = useState('');
  const [model, setModel] = useState('');

  useEffect(() => {
    fetchRobots();
  }, []);

  const fetchRobots = async () => {
    const token = localStorage.getItem('token');

    if (!token) {
      alert('No token found, please log in again.');
      return;
    }

    try {
      const response = await axios.get('http://172.30.1.40:5559/robot/robots', {
        headers: {
          'Authorization': `Bearer ${token}`
        }
      });
      setRobots(response.data);
    } catch (error) {
      console.error('Error fetching robots:', error);
    }
  };

  const handleSelectRobot = (robot) => {
    setSelectedRobot(robot);
    setName(robot.name);
    setIp(robot.ip);
    setModel(robot.model);
  };

  const handleUpdate = async (e) => {
    e.preventDefault();

    const token = localStorage.getItem('token');

    if (!token) {
      alert('No token found, please log in again.');
      return;
    }

    try {
      await axios.put(`http://172.30.1.40:5559/robot/update/${selectedRobot._id}`, {
        name,
        ip,
        model
      }, {
        headers: {
          'Authorization': `Bearer ${token}`,
          'Content-Type': 'application/json'
        }
      });
      alert('Robot updated successfully');
      fetchRobots();
    } catch (error) {
      console.error('Error updating robot:', error);
      alert(`Error updating robot: ${error.response?.data?.message || error.message}`);
    }
  };

  return (
    <div style={{ fontFamily: 'Arial, sans-serif', margin: '20px' }}>
      <header style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '20px' }}>
        <UserInfo />
        <LogoutButton />
      </header>
      <div style={{ display: 'flex' }}>
        <Navbar />
      </div>
      <h2 style={{ textAlign: 'center', margin: '20px 0' }}>Robot Edit</h2>
      <div style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
        <h3>Your Robots</h3>
        <ul style={{ listStyleType: 'none', padding: 0 }}>
          {robots.map((robot) => (
            <li key={robot._id} onClick={() => handleSelectRobot(robot)} style={{ padding: '10px', cursor: 'pointer', borderBottom: '1px solid #ccc' }}>
              {robot.name}
            </li>
          ))}
        </ul>
        {selectedRobot && (
          <form onSubmit={handleUpdate} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
            <input
              type="text"
              placeholder="Robot Name"
              value={name}
              onChange={(e) => setName(e.target.value)}
              required
              style={{ width: '300px', padding: '10px', margin: '10px 0', borderRadius: '5px', border: '1px solid #ccc' }}
            />
            <input
              type="text"
              placeholder="Robot IP"
              value={ip}
              onChange={(e) => setIp(e.target.value)}
              required
              style={{ width: '300px', padding: '10px', margin: '10px 0', borderRadius: '5px', border: '1px solid #ccc' }}
            />
            <input
              type="text"
              placeholder="Robot Model"
              value={model}
              onChange={(e) => setModel(e.target.value)}
              required
              style={{ width: '300px', padding: '10px', margin: '10px 0', borderRadius: '5px', border: '1px solid #ccc' }}
            />
            <button type="submit" style={{ padding: '10px 20px', margin: '20px 0', borderRadius: '5px', backgroundColor: '#4CAF50', color: '#fff', border: 'none', cursor: 'pointer' }}>
              Update Robot
            </button>
          </form>
        )}
      </div>
    </div>
  );
}

export default RobotEditPage;



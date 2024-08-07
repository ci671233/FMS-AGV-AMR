import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';

const MonitoringPage = () => {
  const [mapUrl, setMapUrl] = useState(null);
  const [robots, setRobots] = useState([]);
  const canvasRef = useRef(null);

  useEffect(() => {
    fetchMonitoredMap();
    const ws = new WebSocket('ws://172.30.1.40:5558'); // WebSocket 서버 주소

    ws.onmessage = (event) => {
      const robotPositions = JSON.parse(event.data);
      setRobots(robotPositions);
    };

    return () => {
      ws.close();
    };
  }, []);

  const fetchMonitoredMap = async () => {
    const token = localStorage.getItem('token');
    try {
      const response = await axios.get('http://172.30.1.40:5557/map/monitored/file', {
        headers: { Authorization: `Bearer ${token}` },
        responseType: 'blob',
      });
      const url = URL.createObjectURL(response.data);
      setMapUrl(url);
    } catch (error) {
      console.error('Error fetching monitored map:', error);
    }
  };

  const drawMapAndRobots = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    const mapImage = new Image();
    mapImage.src = mapUrl;

    mapImage.onload = () => {
      canvas.width = mapImage.width;
      canvas.height = mapImage.height;
      ctx.drawImage(mapImage, 0, 0);

      robots.forEach((robot) => {
        const { x, y } = robot.location;
        ctx.fillStyle = 'red';
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, 2 * Math.PI);
        ctx.fill();
      });
    };
  };

  useEffect(() => {
    if (mapUrl) {
      drawMapAndRobots();
    }
  }, [mapUrl, robots]);

  return (
    <div style={{ fontFamily: 'Arial, sans-serif', margin: '20px' }}>
      <header style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '20px' }}>
        <UserInfo />
        <LogoutButton />
      </header>
      <Navbar />
      <h2 style={{ textAlign: 'center', margin: '20px 0' }}>Monitoring Page</h2>
      <div style={{ display: 'flex', justifyContent: 'center', alignItems: 'center', flexDirection: 'column' }}>
        {mapUrl ? (
          <canvas ref={canvasRef} style={{ border: '1px solid #ccc' }}></canvas>
        ) : (
          <p>Loading map...</p>
        )}
      </div>
    </div>
  );
};

export default MonitoringPage;



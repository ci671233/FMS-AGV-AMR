import React, { useState, useEffect, useRef, useCallback } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';
import io from 'socket.io-client';
import SimplePeer from 'simple-peer';

const socket = io.connect('http://172.30.1.40:7000', {
    transports: ['websocket'],
    upgrade: true,
    forceNew: true,
    withCredentials: true
});

function MapCreatePage() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState('');
    const videoRef = useRef();
    const peerRef = useRef(null);
    // const mapRef = useRef(null);

    useEffect(() => {
        const fetchRobots = async () => {
            try {
                const token = localStorage.getItem('token');
                const response = await axios.get('http://172.30.1.40:5559/robot/robots', {
                    headers: { Authorization: `Bearer ${token}` }
                });
                setRobots(response.data);
            } catch (error) {
                console.error('Error fetching robots:', error);
            }
        };

        fetchRobots();

        return () => {
            socket.disconnect();
        };
    }, []);

    useEffect(() => {
        if (selectedRobot) {
            // WebRTC 설정
            const peer = new SimplePeer({
                initiator: true,
                trickle: false
            });

            peer.on('signal', data => {
                socket.emit('signal', { signal: data, robot_id: selectedRobot });
            });

            peer.on('stream', stream => {
                if (videoRef.current) {
                    videoRef.current.srcObject = stream;
                }
            });

            peerRef.current = peer;

            socket.on('signal', data => {
                peer.signal(data.signal);
            });

            // WebSocket 연결을 통해 송신부에 연결
            const connectWebSocket = async () => {
                const ws = new WebSocket('ws://172.30.1.76:8081');  // 송신부의 IP 주소와 8081 포트를 사용
                ws.onopen = () => {
                    console.log('WebSocket connected');
                };
                ws.onmessage = (message) => {
                    const data = JSON.parse(message.data);
                    peer.signal(data);
                };
            };

            connectWebSocket();

            // ROS 연결 설정
            const ros = new window.ROSLIB.Ros({
                url: 'ws://172.30.1.40:9090'  // SLAM 데이터 수신용 ROS WebSocket 서버
            });

            ros.on('connection', () => {
                console.log('Connected to ROS websocket server.');
            });

            ros.on('error', (error) => {
                console.log('Error connecting to ROS websocket server: ', error);
            });

            ros.on('close', () => {
                console.log('Connection to ROS websocket server closed.');
            });

            // 2D 맵을 렌더링하기 위한 ROS2D 설정
            if (window.ROS2D && window.ROSLIB) {
                const viewer = new window.ROS2D.Viewer({
                    divID: 'map',
                    width: 600,
                    height: 600
                });

                const gridClient = new window.ROS2D.OccupancyGridClient({
                    ros: ros,
                    rootObject: viewer.scene,
                    continuous: true
                });

                gridClient.on('change', function() {
                    viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
                    viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
                });

                return () => {
                    ros.close();
                };
            } else {
                console.error('ROS2D is not loaded');
            }
        }
    }, [selectedRobot]);

    const handleKeyDown = useCallback((e) => {
        const velocityCommands = {
            w: { linear: 0.1, angular: 0 },
            a: { linear: 0, angular: 0.1 },
            s: { linear: -0.1, angular: 0 },
            d: { linear: 0, angular: -0.1 },
            ' ': { linear: 0, angular: 0 }
        };
        if (velocityCommands[e.key]) {
            socket.emit('key_press', { robot_id: selectedRobot, velocity: velocityCommands[e.key] });
        }
    }, [selectedRobot]);

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [selectedRobot, handleKeyDown]);

    return (
        <div>
            <header>
                <UserInfo />
                <LogoutButton />
            </header>
            <div style={{ display: 'flex' }}>
                <Navbar />
            </div>
            <h2>SLAM Control</h2>
            <div>
                <label>로봇 선택:</label>
                <select onChange={(e) => setSelectedRobot(e.target.value)} value={selectedRobot}>
                    <option value="">로봇을 선택하세요</option>
                    {robots.map(robot => (
                        <option key={robot._id} value={robot._id}>{robot.name}</option>
                    ))}
                </select>
            </div>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <div style={{ width: '50%' }}>
                    <h3>SLAM 화면</h3>
                    <div id="map" style={{ width: '100%', height: '600px' }} />
                </div>
                <div style={{ width: '50%' }}>
                    <h3>WebCam 화면</h3>
                    <video ref={videoRef} autoPlay playsInline style={{ width: '100%' }} />
                </div>
            </div>
        </div>
    );
}

export default MapCreatePage;






















import React, { useState, useEffect, useRef, useCallback } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';
import ROSLIB from 'roslib';
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
    const mapRef = useRef(null);

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
            if (navigator.mediaDevices && navigator.mediaDevices.getUserMedia) {
                navigator.mediaDevices.getUserMedia({ video: true, audio: false }).then(stream => {
                    if (videoRef.current) {
                        videoRef.current.srcObject = stream;
                    }

                    const peer = new SimplePeer({
                        initiator: true,
                        trickle: false,
                        stream: stream
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
                }).catch(error => console.error('Error accessing media devices.', error));
            } else {
                console.error('MediaDevices API not supported.');
            }

            // ROS 연결 설정
            const ros = new ROSLIB.Ros({
                url: 'ws://172.30.1.40:9090'
            });

            ros.on('connection', () => {
                console.log('Connected to websocket server.');
            });

            ros.on('error', (error) => {
                console.log('Error connecting to websocket server: ', error);
            });

            ros.on('close', () => {
                console.log('Connection to websocket server closed.');
            });

            // SLAM 데이터를 수신하여 화면에 표시
            const mapTopic = new ROSLIB.Topic({
                ros: ros,
                name: '/map',
                messageType: 'nav_msgs/OccupancyGrid'
            });

            mapTopic.subscribe((message) => {
                console.log('Received message on /map: ', message);
                if (mapRef.current) {
                    const canvas = mapRef.current;
                    const ctx = canvas.getContext('2d');
                    const width = message.info.width;
                    const height = message.info.height;
                    canvas.width = width;
                    canvas.height = height;
                    const imageData = ctx.createImageData(width, height);

                    for (let i = 0; i < message.data.length; i++) {
                        const value = message.data[i];
                        const color = value === -1 ? 255 : 255 - value;
                        imageData.data[i * 4] = color;
                        imageData.data[i * 4 + 1] = color;
                        imageData.data[i * 4 + 2] = color;
                        imageData.data[i * 4 + 3] = 255;
                    }

                    ctx.putImageData(imageData, 0, 0);
                }
            });

            return () => {
                ros.close();
            };
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
                    <canvas ref={mapRef} />
                </div>
                <div style={{ width: '50%' }}>
                    <h3>WebCam 화면</h3>
                    <video ref={videoRef} autoPlay playsInline />
                </div>
            </div>
        </div>
    );
}

export default MapCreatePage;




















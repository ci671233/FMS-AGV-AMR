import React, { useState, useEffect, useRef, useCallback } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';
import io from 'socket.io-client';
import SimplePeer from 'simple-peer';

// WebRTC 신호 서버에 연결 (라즈베리파이)
const webcamSocket = io.connect('http://172.30.1.76:7000', { // 시그널링 서버의 IP 주소와 포트
    transports: ['websocket'],
    upgrade: true,
    forceNew: true,
    withCredentials: true
});

// 원격 PC의 SLAM 데이터 수신을 위한 WebSocket 연결
const slamSocket = io.connect('http://172.30.1.40:3000', { // SLAM 데이터를 위한 IP 주소와 포트
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

    // 로봇 목록을 가져오는 부분
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
            webcamSocket.disconnect();
            slamSocket.disconnect();
        };
    }, []);

    // 선택된 로봇의 웹캠 스트림을 수신하는 부분
    useEffect(() => {
        if (selectedRobot) {
            const peer = new SimplePeer({
                initiator: true,
                trickle: false,
                wrtc: window.wrtc
            });

            // 신호 데이터 전송
            peer.on('signal', data => {
                console.log('Sending signal:', data);
                webcamSocket.emit('signal', { signal: data, robot_id: selectedRobot });
            });

            // 스트림을 비디오 요소에 연결
            peer.on('stream', stream => {
                console.log('Stream received:', stream);
                if (videoRef.current) {
                    videoRef.current.srcObject = stream;
                    console.log('Stream set to video element');
                }
            });

            peer.on('error', err => {
                console.error('Peer connection error:', err);
            });

            peer.on('connect', () => {
                console.log('Peer connected');
            });

            peer.on('close', () => {
                console.log('Peer connection closed');
            });

            peer.on('iceStateChange', (state) => {
                console.log('ICE state change:', state);
            });

            peer.on('iceCandidate', (candidate) => {
                console.log('ICE candidate:', candidate);
            });

            peerRef.current = peer;

            // 신호 데이터 수신
            webcamSocket.on('signal', data => {
                console.log('Signal received:', data);
                peer.signal(data.signal);
            });

            webcamSocket.on('connect_error', err => {
                console.error('Webcam socket connection error:', err);
            });

            return () => {
                if (peerRef.current) {
                    peerRef.current.destroy();
                }
            };
        }
    }, [selectedRobot]);

    // ROS 연결 설정 및 SLAM 데이터 처리 부분
    useEffect(() => {
        if (selectedRobot) {
            const ros = new window.ROSLIB.Ros({
                url: 'ws://172.30.1.40:9090'
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

            const mapTopic = new window.ROSLIB.Topic({
                ros: ros,
                name: '/map',
                messageType: 'nav_msgs/OccupancyGrid'
            });

            mapTopic.subscribe((message) => {
                console.log('Received message on /map: ', message);
                if (message.info && message.data) {
                    const { width, height } = message.info;
                    const data = message.data;
                    const canvas = mapRef.current;
                    const context = canvas.getContext('2d');
                    const imageData = context.createImageData(width, height);

                    for (let i = 0; i < data.length; i++) {
                        const value = data[i];
                        const color = value === -1 ? 255 : 255 - value;
                        imageData.data[i * 4] = color;
                        imageData.data[i * 4 + 1] = color;
                        imageData.data[i * 4 + 2] = color;
                        imageData.data[i * 4 + 3] = 255;
                    }

                    context.putImageData(imageData, 0, 0);
                } else {
                    console.error('Received invalid /map message:', message);
                }
            });

            return () => {
                ros.close();
            };
        }
    }, [selectedRobot]);

    // 키보드 입력으로 로봇을 제어하는 부분
    const handleKeyDown = useCallback((e) => {
        const velocityCommands = {
            w: { linear: 0.1, angular: 0 },
            a: { linear: 0, angular: 0.1 },
            s: { linear: -0.1, angular: 0 },
            d: { linear: 0, angular: -0.1 },
            ' ': { linear: 0, angular: 0 }
        };
        if (velocityCommands[e.key]) {
            slamSocket.emit('key_press', { robot_id: selectedRobot, velocity: velocityCommands[e.key] });
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
                    <canvas ref={mapRef} width="600" height="600" />
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




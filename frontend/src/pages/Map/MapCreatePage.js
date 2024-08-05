import React, { useState, useEffect, useRef, useCallback } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';
import Janus from 'janus-gateway';  // npm을 통해 설치한 janus-gateway 라이브러리 로드
import adapter from 'webrtc-adapter';  // webrtc-adapter 라이브러리 로드

function MapCreatePage() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState('');
    const videoRef = useRef();
    const mapRef = useRef(null);
    const rosRef = useRef(null);  // rosRef를 useRef로 정의
    const janusRef = useRef(null);
    const pluginRef = useRef(null);

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
    }, []);

    useEffect(() => {
        if (selectedRobot) {
            Janus.init({ debug: "all", callback: () => {
                janusRef.current = new Janus({
                    server: 'http://localhost:8088/janus',  // Janus 서버 주소
                    success: () => {
                        janusRef.current.attach({
                            plugin: "janus.plugin.videoroom",
                            success: (pluginHandle) => {
                                pluginRef.current = pluginHandle;
                                pluginHandle.send({
                                    message: {
                                        request: "join",
                                        room: 1234,  // 방 ID
                                        ptype: "subscriber",
                                        display: "User"
                                    }
                                });
                            },
                            onmessage: (msg, jsep) => {
                                if (jsep !== undefined && jsep !== null) {
                                    pluginRef.current.createAnswer({
                                        jsep: jsep,
                                        media: { audioSend: false, videoSend: false },
                                        success: (jsep) => {
                                            pluginRef.current.send({
                                                message: { request: "start", room: 1234 },
                                                jsep: jsep
                                            });
                                        },
                                        error: (error) => {
                                            console.error("WebRTC error:", error);
                                        }
                                    });
                                }
                            },
                            onremotestream: (stream) => {
                                if (videoRef.current) {
                                    videoRef.current.srcObject = stream;
                                }
                            }
                        });
                    },
                    error: (error) => {
                        console.error("Janus error:", error);
                    }
                });
            }});
        }
    }, [selectedRobot]);

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

            rosRef.current = ros;  // rosRef 초기화

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
        if (velocityCommands[e.key] && rosRef.current) {  // rosRef 사용
            const cmdVel = new window.ROSLIB.Topic({
                ros: rosRef.current,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            });

            const twist = new window.ROSLIB.Message({
                linear: {
                    x: velocityCommands[e.key].linear,
                    y: 0,
                    z: 0
                },
                angular: {
                    x: 0,
                    y: 0,
                    z: velocityCommands[e.key].angular
                }
            });

            cmdVel.publish(twist);
        }
    }, []);

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [handleKeyDown]);

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













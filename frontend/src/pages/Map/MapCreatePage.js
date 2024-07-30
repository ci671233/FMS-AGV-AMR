import React, { useState, useEffect, useRef, useCallback } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';
import io from 'socket.io-client';
import SimplePeer from 'simple-peer';

const socket = io.connect('http://172.30.1.40:7000'); // WebSocket 서버 주소

function MapCreatePage() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState('');
    const videoRef = useRef();
    const peerRef = useRef(null);

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

        return () => {
            socket.disconnect();
        };
    }, [selectedRobot]);

    const sendCommand = (command) => {
        const token = localStorage.getItem('token');
        fetch('http://172.30.1.40:5559/robot/send_command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}`
            },
            body: JSON.stringify({
                robot_id: selectedRobot,
                command: command
            })
        }).then(response => response.text())
            .then(data => alert(data))
            .catch(error => console.error('Error sending command:', error));
    };

    const handleKeyDown = useCallback((e) => {
        const velocityCommands = {
            w: { linear: 0.1, angular: 0 },
            a: { linear: 0, angular: 0.1 },
            s: { linear: -0.1, angular: 0 },
            d: { linear: 0, angular: -0.1 },
            ' ': { linear: 0, angular: 0 }
        };
        if (velocityCommands[e.key]) {
            if (peerRef.current) {
                peerRef.current.send(JSON.stringify({
                    robot_id: selectedRobot,
                    velocity: velocityCommands[e.key]
                }));
            }
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
            <button onClick={() => sendCommand('slam')}>SLAM 시작</button>
            <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                <div style={{ width: '50%' }}>
                    <h3>SLAM 화면</h3>
                    <img src="http://172.30.1.40:8080/stream?topic=/camera/rgb/image_raw" alt="SLAM View" />
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








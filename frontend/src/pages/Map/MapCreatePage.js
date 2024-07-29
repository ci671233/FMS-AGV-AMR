import React, { useState, useEffect } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MapCreatePage() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState('');

    useEffect(() => {
        // 페이지 로드 시 로봇 목록을 가져옴
        const fetchRobots = async () => {
            try {
                const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴
                const response = await axios.get('http://172.30.1.40:5559/robot/robots', {
                    headers: {
                        Authorization: `Bearer ${token}`
                    }
                });
                setRobots(response.data);
            } catch (error) {
                console.error('Error fetching robots:', error);
            }
        };

        fetchRobots();

        // WebSocket 연결 설정
        const ws = new WebSocket(`ws://${window.location.hostname}:8080`); // WebSocket 서버 주소
        ws.onopen = () => {
            console.log('WebSocket 연결이 설정되었습니다.');
        };

        const handleKeyDown = (e) => {
            const velocityCommands = {
                w: { linear: 0.1, angular: 0 },
                a: { linear: 0, angular: 0.1 },
                s: { linear: -0.1, angular: 0 },
                d: { linear: 0, angular: -0.1 },
                ' ': { linear: 0, angular: 0 } // space 키로 변경
            };
            if (velocityCommands[e.key] && selectedRobot) {
                ws.send(JSON.stringify({
                    robot_id: selectedRobot,
                    velocity: velocityCommands[e.key]
                }));
            }
        };

        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
            ws.close(); // 컴포넌트 언마운트 시 WebSocket 연결 해제
        };
    }, [selectedRobot]);

    // 서버에 명령을 전송하는 함수
    const sendCommand = (command) => {
        const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴
        fetch('http://172.30.1.40:5559/robot/send_command', { // 절대 경로 사용
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
                'Authorization': `Bearer ${token}` // 토큰을 헤더에 추가
            },
            body: JSON.stringify({
                robot_id: selectedRobot,
                command: command
            })
        }).then(response => response.text())
            .then(data => alert(data))
            .catch(error => console.error('Error sending command:', error));
    };

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
            <div>
                <img src={`http://${selectedRobot}:8080/stream?topic=/camera/rgb/image_raw`} alt="SLAM View" />
            </div>
        </div>
    );
};

export default MapCreatePage;



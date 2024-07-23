import React, { useState, useEffect } from 'react'; // useEffect 추가
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MapCreatePage() {
    const [robots, setRobots] = useState([]);
    const [selectedRobot, setSelectedRobot] = useState('');
    const ws = new WebSocket('ws://localhost:3000'); // WebSocket 서버 주소
  
    useEffect(() => {
        // 페이지 로드 시 로봇 목록을 가져옴
        const fetchRobots = async () => {
            try {
                const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴
                const response = await axios.get('http://localhost:5557/robots', {
                    headers: {
                        Authorization: `Bearer ${token}`
                    }
                });
                setRobots(response.data); // setMaps 대신 setRobots 사용
            } catch (error) {
                console.error('Error fetching robots:', error);
            }
        };

        fetchRobots();

        // WebSocket 연결 설정
        ws.onopen = () => {
            console.log('WebSocket 연결이 설정되었습니다.');
        };
    }, []); // 빈 배열을 전달하여 컴포넌트가 처음 렌더링될 때만 실행되도록 함

    // 서버에 명령을 전송하는 함수
    const sendCommand = (command) => {
        fetch('/send_command', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                robot_id: selectedRobot,
                command: command
            })
        }).then(response => response.text())
            .then(data => alert(data));
    };

    // 키 입력을 처리하여 로봇을 제어하는 함수
    const handleKeyDown = (e) => {
        const velocityCommands = {
            w: { linear: 0.1, angular: 0 },
            a: { linear: 0, angular: 0.1 },
            s: { linear: -0.1, angular: 0 },
            d: { linear: 0, angular: -0.1 },
            x: { linear: 0, angular: 0 }
        };
        if (velocityCommands[e.key]) {
            ws.send(JSON.stringify({
                robot_id: selectedRobot,
                velocity: velocityCommands[e.key]
            }));
        }
    };

    useEffect(() => {
        window.addEventListener('keydown', handleKeyDown);
        return () => {
            window.removeEventListener('keydown', handleKeyDown);
        };
    }, [selectedRobot]); // selectedRobot이 변경될 때마다 실행되도록 설정

    return (
        <div>
            <header>
                <UserInfo />
                <LogoutButton />
            </header>
            <div style={{ display: 'flex' }}>
                <Navbar />
            </div>
            <h2>Turtlebot3 SLAM Control</h2>
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
                <img src="http://localhost:8080/stream?topic=/camera/rgb/image_raw" alt="SLAM View" />
            </div>
        </div>
    );
};

export default MapCreatePage;


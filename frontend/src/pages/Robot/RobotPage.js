import React, { useState } from 'react'; 
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function RobotPage() {
    const [name, setName] = useState('');
    const [ip, setIp] = useState('');
    const [model, setModel] = useState('');

    const handleUpload = async (e) => {
        e.preventDefault();
        const formData = {
            name: name,
            ip: ip,
            model: model
        };

        const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴

        if (!token) {
            alert('No token found, please log in again.');
            return;
        }

        try {
            await axios.post('http://172.30.1.40:5559/robot/register_robot', formData, {
                headers: { 
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${token}` 
                }
            });
            alert('Robot registered successfully');
        } catch (error) {
            console.error('Error registering robot:', error);
            alert('Failed to register robot');
        }
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
          <h2>Robot Register</h2>
          <form onSubmit={handleUpload}>
            <input
                type="text"
                placeholder="Robot Name"
                value={name}
                onChange={(e) => setName(e.target.value)}
                required
            />
            <input
                type="text"
                placeholder="Robot IP"
                value={ip}
                onChange={(e) => setIp(e.target.value)}
                required
            />
            <input
                type="text"
                placeholder="Robot Model"
                value={model}
                onChange={(e) => setModel(e.target.value)}
                required
            />
            <button type="submit">Register Robot</button>
          </form>
        </div>
    );
}

export default RobotPage;

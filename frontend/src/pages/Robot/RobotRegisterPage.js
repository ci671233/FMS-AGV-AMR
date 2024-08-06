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

        const token = localStorage.getItem('token');

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
        <div style={{ fontFamily: 'Arial, sans-serif', margin: '20px' }}>
            <header style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '20px' }}>
                <UserInfo />
                <LogoutButton />
            </header>
            <div style={{ display: 'flex' }}>
                <Navbar />
            </div>
            <h2 style={{ textAlign: 'center', margin: '20px 0' }}>Robot Register</h2>
            <form onSubmit={handleUpload} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
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
                    Register Robot
                </button>
            </form>
        </div>
    );
}

export default RobotPage;

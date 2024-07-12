import React from 'react';
import { useNavigate } from 'react-router-dom';

function LogoutButton() {
    const navigate = useNavigate();

    const handleLogout = () => {
        localStorage.removeItem('token');
        navigate('/account/login');
    };

    return <button onClick={handleLogout}>Logout</button>;
}

export default LogoutButton;

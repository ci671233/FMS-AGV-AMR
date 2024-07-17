import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import LoginPage from './pages/Account/LoginPage';
import RegisterPage from './pages/Account/RegisterPage';
import MainPage from './pages/Main/MainPage';
import MonitorPage from './pages/Monitor/MonitoringPage';
import ControlPage from './pages/Control/ControlPage';
import RobotPage from './pages/Robot/RobotPage';
import MapCreatePage from './pages/Map/MapCreatePage';
import MapEditPage from './pages/Map/MapEditPage';
import LogPage from './pages/Log/LogPage';

function App() {
    return (
        <Router>
            <Routes>
                <Route path="/login" element={<LoginPage />} />
                <Route path="/register" element={<RegisterPage />} />
                <Route path="/main/*" element={<MainPage />} />
                <Route path="/monitor" element={<MonitorPage />} />
                <Route path="/control" element={<ControlPage />} />
                <Route path="/robot" element={<RobotPage />} />
                <Route path="/map_create" element={<MapCreatePage />} />
                <Route path="/map_edit" element={<MapEditPage />} />
                <Route path="/logs" element={<LogPage />} />
                <Route path="/" element={<LoginPage />} /> {/* 기본 경로는 로그인 페이지로 설정 */}
            </Routes>
        </Router>
    );
}

export default App;



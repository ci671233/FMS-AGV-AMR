import React from 'react';
import { BrowserRouter as Router, Routes, Route } from 'react-router-dom';
import LoginPage from './pages/Account/LoginPage';
import RegisterPage from './pages/Account/RegisterPage';
import MainPage from './pages/Main/MainPage';

function App() {
    return (
        <Router>
            <Routes>
                <Route path="/login" element={<LoginPage />} />
                <Route path="/register" element={<RegisterPage />} />
                <Route path="/main/*" element={<MainPage />} />
                <Route path="/" element={<LoginPage />} /> {/* 기본 경로는 로그인 페이지로 설정 */}
            </Routes>
        </Router>
    );
}

export default App;


import React from 'react';
import { Routes, Route } from 'react-router-dom';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/UserInfo';
import MonitoringPage from './MonitoringPage';
import ControlPage from './ControlPage';
import MapPage from './MapPage';

function MainPage() {
    return (
        <div>
            <header>
                <UserInfo />
                <LogoutButton />
            </header>
            <div style={{ display: 'flex' }}>
                <Navbar />
                <main style={{ marginLeft: '200px', padding: '20px' }}>
                    <Routes>
                        <Route path="monitoring" element={<MonitoringPage />} />
                        <Route path="control" element={<ControlPage />} />
                        <Route path="map" element={<MapPage />} />
                    </Routes>
                </main>
            </div>
        </div>
    );
}

export default MainPage;

import React from 'react';
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import { AuthProvider } from './context/AuthContext';
import Login from './components/Login';
import Register from './components/Register';
// import AdminDashboard from './components/AdminDashboard';
// import UserManagement from './components/UserManagement';
// import FactoryManagement from './components/FactoryManagement';
// import Settings from './components/Settings';
// import FactoryList from './components/FactoryList';
// import FactoryDetails from './components/FactoryDetails';
// import Monitoring from './components/Monitoring';
// import Logs from './components/Logs';
// import Map from './components/Map';
// import Robot from './components/Robot';
// import CreateFactory from './components/CreateFactory';

function App() {
  return (
    <AuthProvider>
      <Router>
        <div>
          <Routes>
            <Route path="/login" element={<Login />} />
            <Route path="/register" element={<Register />} />
            {/* <Route path="/admin" element={<AdminDashboard />} />
            <Route path="/admin/user-management" element={<UserManagement />} />
            <Route path="/admin/factory-management" element={<FactoryManagement />} />
            <Route path="/admin/settings" element={<Settings />} />
            <Route path="/factories" element={<FactoryList />} />
            <Route path="/factory/:id" element={<FactoryDetails />} />
            <Route path="/factory/:id/monitoring" element={<Monitoring />} />
            <Route path="/factory/:id/logs" element={<Logs />} />
            <Route path="/factory/:id/map" element={<Map />} />
            <Route path="/factory/:id/robot" element={<Robot />} />
            <Route path="/factory/:id/settings" element={<Settings />} />
            <Route path="/create-factory" element={<CreateFactory />} /> */}
          </Routes>
        </div>
      </Router>
    </AuthProvider>
  );
}

export default App;




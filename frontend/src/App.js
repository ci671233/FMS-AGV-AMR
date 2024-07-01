import React from 'react';
import { BrowserRouter as Router, Route, Routes, Link } from 'react-router-dom';
import Register from './components/Register';
import Login from './components/Login';
import NewAccessToken from './components/NewAccessToken';
import ApproveAccount from './components/ApproveAccount';
import AssignRole from './components/AssignRole';
import CreateFactory from './components/CreateFactory';
import UpdateFactory from './components/UpdateFactory';
import AddUserToFactory from './components/AddUserToFactory';
import AddAdminToFactory from './components/AddAdminToFactory';

function App() {
  return (
    <Router>
      <div>
        <nav>
          <ul>
            {/* 링크버튼 */}
            <li><Link to="/register">Register</Link></li> 
            <li><Link to="/login">Login</Link></li>
            <li><Link to="/newaccesstoken">New Access Token</Link></li>
            <li><Link to="/approveaccount">Approve Account</Link></li>
            <li><Link to="/assignrole">Assign Role</Link></li>
            <li><Link to="/createfactory">Create Factory</Link></li>
            <li><Link to="/updatefactory">Update Factory</Link></li>
            <li><Link to="/addusertofactory">Add User to Factory</Link></li>
            <li><Link to="/addadmintofactory">Add Admin to Factory</Link></li>
          </ul>
        </nav>
        <Routes>
          {/* 라우트 설정 */}
          <Route path="/register" element={<Register />} />
          <Route path="/login" element={<Login />} />
          <Route path="/newaccesstoken" element={<NewAccessToken />} />
          <Route path="/approveaccount" element={<ApproveAccount />} />
          <Route path="/assignrole" element={<AssignRole />} />
          <Route path="/createfactory" element={<CreateFactory />} />
          <Route path="/updatefactory" element={<UpdateFactory />} />
          <Route path="/addusertofactory" element={<AddUserToFactory />} />
          <Route path="/addadmintofactory" element={<AddAdminToFactory />} />
        </Routes>
      </div>
    </Router>
  );
}

export default App;


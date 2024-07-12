import React from 'react';
import { Link } from 'react-router-dom';

function AdminDashboard() {
  return (
    <div>
      <nav>
        <ul>
          <li><Link to="/admin/user-management">User Management</Link></li>
          <li><Link to="/admin/factory-management">Factory Management</Link></li>
          <li><Link to="/admin/settings">Settings</Link></li>
        </ul>
      </nav>
    </div>
  );
}

export default AdminDashboard;

import React from 'react';
import { Outlet, Link } from 'react-router-dom';

function Admin() {
  return (
    <div>
      <nav>
        <ul>
          <li>
            <Link to="/admin/user-management">User Management</Link>
          </li>
          <li>
            <Link to="/admin/factory-management">Factory Management</Link>
          </li>
          <li>
            <Link to="/admin/setting">Setting</Link>
          </li>
        </ul>
      </nav>
      <div>
        <Outlet />
      </div>
    </div>
  );
}

export default Admin;


import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function AssignRole() {
  const [accountID, setAccountID] = useState('');
  const [role, setRole] = useState('');
  const { token } = useContext(AuthContext);

  const handleAssignRole = async () => {
    try {
      const response = await axios.put(`http://172.30.1.28:8080/api/admin/assign_role/${accountID}`, { role }, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('Role assigned successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Error assigning role:', error);
      alert('Failed to assign role.');
    }
  };

  return (
    <div>
      <h2>Assign Role</h2>
      <input type="text" placeholder="Account ID" value={accountID} onChange={(e) => setAccountID(e.target.value)} />
      <input type="text" placeholder="Role" value={role} onChange={(e) => setRole(e.target.value)} />
      <button onClick={handleAssignRole}>Assign Role</button>
    </div>
  );
}

export default AssignRole;

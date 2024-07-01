import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function AddAdminToFactory() {
  const [factoryID, setFactoryID] = useState('');
  const [adminID, setAdminID] = useState('');
  const { token } = useContext(AuthContext);

  const handleAddAdminToFactory = async () => {
    try {
      const response = await axios.put(`http://172.30.1.28:8080/api/factory/add_admin/${factoryID}/${adminID}`, {}, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('Admin added to factory successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Error adding admin to factory:', error);
      alert('Failed to add admin to factory.');
    }
  };

  return (
    <div>
      <h2>Add Admin to Factory</h2>
      <input type="text" placeholder="Factory ID" value={factoryID} onChange={(e) => setFactoryID(e.target.value)} />
      <input type="text" placeholder="Admin ID" value={adminID} onChange={(e) => setAdminID(e.target.value)} />
      <button onClick={handleAddAdminToFactory}>Add Admin to Factory</button>
    </div>
  );
}

export default AddAdminToFactory;

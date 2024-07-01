import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function AddUserToFactory() {
  const [factoryID, setFactoryID] = useState('');
  const [userID, setUserID] = useState('');
  const { token } = useContext(AuthContext);

  const handleAddUserToFactory = async () => {
    try {
      const response = await axios.put(`http://172.30.1.28:8080/api/factory/add_user/${factoryID}/${userID}`, {}, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('User added to factory successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Error adding user to factory:', error);
      alert('Failed to add user to factory.');
    }
  };

  return (
    <div>
      <h2>Add User to Factory</h2>
      <input type="text" placeholder="Factory ID" value={factoryID} onChange={(e) => setFactoryID(e.target.value)} />
      <input type="text" placeholder="User ID" value={userID} onChange={(e) => setUserID(e.target.value)} />
      <button onClick={handleAddUserToFactory}>Add User to Factory</button>
    </div>
  );
}

export default AddUserToFactory;


import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function UpdateFactory() {
  const [factoryID, setFactoryID] = useState('');
  const [updates, setUpdates] = useState({});
  const { token } = useContext(AuthContext);

  const handleUpdateFactory = async () => {
    try {
      const response = await axios.put(`http://172.30.1.28:8080/api/factory/update/${factoryID}`, updates, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('Factory updated successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Error updating factory:', error);
      alert('Failed to update factory.');
    }
  };

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setUpdates({
      ...updates,
      [name]: value
    });
  };

  return (
    <div>
      <h2>Update Factory</h2>
      <input type="text" placeholder="Factory ID" value={factoryID} onChange={(e) => setFactoryID(e.target.value)} />
      <input type="text" name="name" placeholder="Name" onChange={handleInputChange} />
      <input type="text" name="location" placeholder="Location" onChange={handleInputChange} />
      <input type="text" name="size" placeholder="Size" onChange={handleInputChange} />
      <input type="text" name="description" placeholder="Description" onChange={handleInputChange} />
      <button onClick={handleUpdateFactory}>Update Factory</button>
    </div>
  );
}

export default UpdateFactory;

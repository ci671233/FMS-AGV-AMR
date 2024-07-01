import React, { useState, useContext } from 'react';
import axios from 'axios';
import AuthContext from '../context/AuthContext';

function CreateFactory() {
  const [name, setName] = useState('');
  const [location, setLocation] = useState('');
  const [size, setSize] = useState('');
  const [description, setDescription] = useState('');
  const { token } = useContext(AuthContext);

  const handleCreateFactory = async () => {
    try {
      const response = await axios.post('http://172.30.1.28:8080/api/factory/create', {
        name,
        location,
        size,
        description
      }, {
        headers: { Authorization: `Bearer ${token}` }
      });
      alert('Factory created successfully.');
      console.log(response.data);
    } catch (error) {
      console.error('Error creating factory:', error);
      alert('Failed to create factory.');
    }
  };

  return (
    <div>
      <h2>Create Factory</h2>
      <input type="text" placeholder="Name" value={name} onChange={(e) => setName(e.target.value)} />
      <input type="text" placeholder="Location" value={location} onChange={(e) => setLocation(e.target.value)} />
      <input type="text" placeholder="Size" value={size} onChange={(e) => setSize(e.target.value)} />
      <input type="text" placeholder="Description" value={description} onChange={(e) => setDescription(e.target.value)} />
      <button onClick={handleCreateFactory}>Create Factory</button>
    </div>
  );
}

export default CreateFactory;


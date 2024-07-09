import React, { useState } from 'react';
import axios from 'axios';

function CreateFactory() {
  const [name, setName] = useState('');
  const [location, setLocation] = useState('');
  const [size, setSize] = useState('');
  const [message, setMessage] = useState('');

  const handleCreateFactory = async () => {
    try {
      const response = await axios.post(`${process.env.REACT_APP_API_URL}/factory/create`, {
        name,
        location,
        size,
      });
      setMessage('Factory creation request successful. Waiting for approval.');
      console.log(response.data);
    } catch (error) {
      console.error('Factory creation error:', error.response || error);
      setMessage(`Factory creation failed: ${error.response?.data?.message || error.message}`);
    }
  };

  return (
    <div>
      <h2>Create Factory</h2>
      <input type="text" placeholder="Factory Name" value={name} onChange={(e) => setName(e.target.value)} />
      <input type="text" placeholder="Factory Location" value={location} onChange={(e) => setLocation(e.target.value)} />
      <input type="text" placeholder="Factory Size" value={size} onChange={(e) => setSize(e.target.value)} />
      <button onClick={handleCreateFactory}>Create Factory</button>
      {message && <p>{message}</p>}
    </div>
  );
}

export default CreateFactory;

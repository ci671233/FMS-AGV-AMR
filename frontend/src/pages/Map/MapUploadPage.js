import React, { useState } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MapUploadPage() {
  const [name, setName] = useState('');
  const [description, setDescription] = useState('');
  const [file, setFile] = useState(null);

  const handleFile = (e) => {
    setFile(e.target.files[0]);
  };

  const handleUpload = async (e) => {
    e.preventDefault();
    const formData = new FormData();
    formData.append('file', file);
    formData.append('name', name);
    formData.append('description', description);

    try {
      await axios.post('http://localhost:5557/map/upload', formData, {
        headers: { 'Content-Type': 'multipart/form-data' }
      });
      alert('Map uploaded successfully');
    } catch (error) {
      console.error('Error uploading map:', error);
    }
  };

  return (
    <div style={{ fontFamily: 'Arial, sans-serif', margin: '20px' }}>
      <header style={{ display: 'flex', justifyContent: 'space-between', marginBottom: '20px' }}>
        <UserInfo />
        <LogoutButton />
      </header>
      <div style={{ display: 'flex' }}>
        <Navbar />
      </div>
      <h2 style={{ textAlign: 'center', margin: '20px 0' }}>Map Create</h2>
      <form onSubmit={handleUpload} style={{ display: 'flex', flexDirection: 'column', alignItems: 'center' }}>
        <input
          type="text"
          placeholder="Map Name"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
          style={{ width: '300px', padding: '10px', margin: '10px 0', borderRadius: '5px', border: '1px solid #ccc' }}
        />
        <textarea
          placeholder="Map Description"
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          required
          style={{ width: '300px', padding: '10px', margin: '10px 0', borderRadius: '5px', border: '1px solid #ccc' }}
        />
        <label htmlFor="file-upload" style={{ margin: '10px 0', color: '#333' }}>파일을 올려주세요</label>
        <input
          id="file-upload"
          type="file"
          accept="image/*"
          onChange={handleFile}
          required
          style={{ margin: '10px 0' }}
        />
        <button type="submit" style={{ padding: '10px 20px', margin: '20px 0', borderRadius: '5px', backgroundColor: '#4CAF50', color: '#fff', border: 'none', cursor: 'pointer' }}>
          Upload Map
        </button>
      </form>
    </div>
  );
}

export default MapUploadPage;


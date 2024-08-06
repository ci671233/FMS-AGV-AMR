import React, { useState } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MapUploadPage() {
  const [name, setName] = useState('');
  const [description, setDescription] = useState('');
  const [pngFile, setPngFile] = useState(null);

  const handlePngFile = (e) => {
    setPngFile(e.target.files[0]);
  };

  const handleUpload = async (e) => {
    e.preventDefault();
    const formData = new FormData();
    formData.append('pgm', pngFile);
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
    <div>
      <header>
            <UserInfo />
            <LogoutButton />
      </header>
      <div style={{ display: 'flex' }}>
                <Navbar />
            </div>
      <h2>Map Create</h2>
      <form onSubmit={handleUpload}>
        <input
          type="text"
          placeholder="Map Name"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
        />
        <textarea
          placeholder="Map Description"
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          required
        />
        <label>png 파일을 올려주세요</label>
        <input
          type="file"
          accept=".png"
          onChange={handlePngFile}
          required
        />
        <button type="submit">Upload Map</button>
      </form>
    </div>
  );
}

export default MapUploadPage;


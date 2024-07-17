import React, { useState } from 'react';
import axios from 'axios';

function MapCreatePage() {
  const [name, setName] = useState('');
  const [description, setDescription] = useState('');
  const [pgmFile, setPgmFile] = useState(null);
  const [yamlFile, setYamlFile] = useState(null);

  const handlePgmFileChange = (e) => {
    setPgmFile(e.target.files[0]);
  };

  const handleYamlFileChange = (e) => {
    setYamlFile(e.target.files[0]);
  };

  const handleUpload = async (e) => {
    e.preventDefault();
    const formData = new FormData();
    formData.append('pgm', pgmFile);
    formData.append('yaml', yamlFile);
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
        <label>pgm 파일을 올려주세요</label>
        <input
          type="file"
          accept=".pgm"
          onChange={handlePgmFileChange}
          required
        />
        <label>yaml 파일을 올려주세요</label>
        <input
          type="file"
          accept=".yaml"
          onChange={handleYamlFileChange}
          required
        />
        <button type="submit">Upload Map</button>
      </form>
    </div>
  );
}

export default MapCreatePage;


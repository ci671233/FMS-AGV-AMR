import React, { useState, useEffect } from 'react';
import axios from 'axios';

function MapPage() {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [name, setName] = useState('');
  const [description, setDescription] = useState('');
  const [file, setFile] = useState(null);

  useEffect(() => {
    const fetchMaps = async () => {
      try {
        const response = await axios.get('http://localhost:5557/map');
        setMaps(response.data);
      } catch (error) {
        console.error('Error fetching maps:', error);
      }
    };

    fetchMaps();
  }, []);

  const handleMapSelect = (map) => {
    setSelectedMap(map);
  };

  const handleFileChange = (e) => {
    const selectedFile = e.target.files[0];
    setFile(selectedFile);
  };

  const handleUpload = async (e) => {
    e.preventDefault();
    const formData = new FormData();
    formData.append('file', file);
    formData.append('name', name);
    formData.append('description', description); // Description 정보를 문자열로 전송

    try {
      await axios.post('http://localhost:5557/map/upload', formData, {
        headers: { 'Content-Type': 'multipart/form-data' }
      });
      alert('Map uploaded successfully');
      const response = await axios.get('http://localhost:5557/map');
      setMaps(response.data);
    } catch (error) {
      console.error('Error uploading map:', error);
    }
  };

  const handleDownload = async (mapId) => {
    try {
      const response = await axios.get(`http://localhost:5557/map/download/${mapId}`, {
        responseType: 'blob'
      });

      const url = window.URL.createObjectURL(new Blob([response.data]));
      const link = document.createElement('a');
      link.href = url;
      link.setAttribute('download', selectedMap.name); // 파일 이름 설정
      document.body.appendChild(link);
      link.click();
    } catch (error) {
      console.error('Error downloading map:', error);
    }
  };

  return (
    <div>
      <h2>Map Editor</h2>
      <ul>
        {maps.map(map => (
          <li key={map._id} onClick={() => handleMapSelect(map)}>
            {map.name}
          </li>
        ))}
      </ul>
      {selectedMap && (
        <div>
          <h3>Edit Map: {selectedMap.name}</h3>
          <button onClick={() => handleDownload(selectedMap._id)}>Download Map</button>
        </div>
      )}
      <form onSubmit={handleUpload}>
        <input
          type="text"
          placeholder="Map Name"
          value={name}
          onChange={(e) => setName(e.target.value)}
          required
        />
        <input
          type="file"
          accept=".pgm, .yaml"
          onChange={handleFileChange}
          required
        />
        <textarea
          placeholder="Map Description"
          value={description}
          onChange={(e) => setDescription(e.target.value)}
          required
        />
        <button type="submit">Upload Map</button>
      </form>
    </div>
  );
}

export default MapPage;

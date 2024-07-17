import React, { useState, useEffect } from 'react';
import Navbar from '../../components/Common/Navbar';
import axios from 'axios';

function MapEditPage() {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [monitoringMapId, setMonitoringMapId] = useState('');
  const [mapImage, setMapImage] = useState('');

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

  const handleMapSelect = async (map) => {
    setSelectedMap(map);
    try {
      const response = await axios.get(`http://localhost:5557/map/png/${map._id}`, {
        responseType: 'arraybuffer'
      });
      const base64Image = Buffer.from(response.data, 'binary').toString('base64');
      setMapImage(`data:image/png;base64,${base64Image}`);
    } catch (error) {
      console.error('Error fetching map image:', error);
    }
  };

  const handleMonitoringMapSelect = async (mapId) => {
    const token = localStorage.getItem('token'); // 토큰 가져오기
    try {
      await axios.post('http://localhost:5557/map/select', { mapId }, {
        headers: { Authorization: `Bearer ${token}` } // 인증 헤더 추가
      });
      setMonitoringMapId(mapId);
      alert('Map selected for monitoring successfully');
    } catch (error) {
      console.error('Error selecting map for monitoring:', error);
    }
  };

  return (
    <div>
      <Navbar />
      <h2>Map Edit</h2>
      <ul>
        {maps.map((map) => (
          <li 
            key={map._id} 
            onClick={() => handleMapSelect(map)}
            style={{ cursor: 'pointer', fontWeight: map._id === monitoringMapId ? 'bold' : 'normal' }}
          >
            {map.name}
          </li>
        ))}
      </ul>
      {selectedMap && (
        <div>
          <h3>Map Details: {selectedMap.name}</h3>
          <p>Description: {selectedMap.description}</p>
          {mapImage && <img src={mapImage} alt="Selected Map" style={{ maxWidth: '100%' }} />}
          <button onClick={() => handleMonitoringMapSelect(selectedMap._id)}>
            {selectedMap._id === monitoringMapId ? 'Selected for Monitoring' : 'Select for Monitoring'}
          </button>
        </div>
      )}
    </div>
  );
}

export default MapEditPage;


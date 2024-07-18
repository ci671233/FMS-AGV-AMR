import React, { useState, useEffect } from 'react';
import Navbar from '../../components/Common/Navbar';
import LogoutButton from '../../components/Common/LogoutButton';
import UserInfo from '../../components/Common/UserInfo';
import axios from 'axios';

function MapEditPage() {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [monitoringMapId, setMonitoringMapId] = useState('');
  const [mapImage, setMapImage] = useState('');

  useEffect(() => {
    const fetchMaps = async () => {
      try {
        const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴
        const response = await axios.get('http://localhost:5557/map', {
          headers: {
            Authorization: `Bearer ${token}`
          }
        });
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
      const token = localStorage.getItem('token'); // Add token to the request
      const response = await axios.get(`http://localhost:5557/map/png/${map._id}`, {
        headers: {
          Authorization: `Bearer ${token}`
        },
        responseType: 'arraybuffer'
      });
      const base64Image = Buffer.from(response.data, 'binary').toString('base64');
      setMapImage(`data:image/png;base64,${base64Image}`);
    } catch (error) {
      console.error('Error fetching map image:', error);
    }
  };
  

  const handleMonitoringMapSelect = async (mapId) => {
    try {
      const token = localStorage.getItem('token'); // 로그인 시 저장한 토큰을 가져옴
      if (!token) {
        console.error('No token found in localStorage');
        return;
      }
      const userId = JSON.parse(atob(token.split('.')[1])).id; // 토큰에서 사용자 ID 추출
      await axios.post('http://localhost:5557/map/select', { mapId, userId }, {
        headers: {
          Authorization: `Bearer ${token}`
        }
      });
      setMonitoringMapId(mapId);
      alert('Map selected for monitoring successfully');
    } catch (error) {
      console.error('Error selecting map for monitoring:', error);
    }
  };
  

  return (
    <div>
      <header>
          <UserInfo />
          <LogoutButton />
      </header>
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


import React, { useState, useEffect } from 'react';
import axios from 'axios';

function MapEditPage() {
  const [maps, setMaps] = useState([]);
  const [selectedMap, setSelectedMap] = useState(null);
  const [monitoringMapId, setMonitoringMapId] = useState('');

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

  const handleMonitoringMapSelect = async (mapId) => {
    try {
      await axios.post('http://localhost:5557/map/select', { mapId });
      setMonitoringMapId(mapId);
      alert('Map selected for monitoring successfully');
    } catch (error) {
      console.error('Error selecting map for monitoring:', error);
    }
  };

  return (
    <div>
      <h2>Map Edit</h2>
      <ul>
        {maps.map((map) => (
          <li key={map._id} onClick={() => handleMapSelect(map)}>
            {map.name}
          </li>
        ))}
      </ul>
      {selectedMap && (
        <div>
          <h3>Map Details: {selectedMap.name}</h3>
          <p>Description: {selectedMap.description}</p>
          <button onClick={() => handleMonitoringMapSelect(selectedMap._id)}>
            Select for Monitoring
          </button>
        </div>
      )}
    </div>
  );
}

export default MapEditPage;

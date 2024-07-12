import React, { useState, useEffect } from 'react';
import { useParams } from 'react-router-dom';
import axios from 'axios';

function Map() {
  const { id } = useParams();
  const [mapData, setMapData] = useState(null);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchMapData = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/factory/${id}/map`);
        setMapData(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchMapData();
  }, [id]);

  return (
    <div>
      <h2>Map</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <div>
        <h3>Map Data</h3>
        {/* 여기에 맵 데이터를 렌더링하는 로직을 추가합니다. */}
      </div>
    </div>
  );
}

export default Map;

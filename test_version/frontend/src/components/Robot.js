import React, { useState, useEffect } from 'react';
import { useParams } from 'react-router-dom';
import axios from 'axios';

function Robot() {
  const { id } = useParams();
  const [robots, setRobots] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchRobots = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/factory/${id}/robots`);
        setRobots(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchRobots();
  }, [id]);

  return (
    <div>
      <h2>Robot</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <ul>
        {robots.map((robot) => (
          <li key={robot._id}>
            {robot.name} - {robot.model}
            {/* 로봇 이동 명령을 추가하는 버튼 등을 추가할 수 있습니다. */}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default Robot;

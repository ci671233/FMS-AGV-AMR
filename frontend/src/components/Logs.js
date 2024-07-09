import React, { useState, useEffect } from 'react';
import { useParams } from 'react-router-dom';
import axios from 'axios';

function Logs() {
  const { id } = useParams();
  const [logs, setLogs] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchLogs = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/factory/${id}/logs`);
        setLogs(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchLogs();
  }, [id]);

  return (
    <div>
      <h2>Logs</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <ul>
        {logs.map((log) => (
          <li key={log._id}>
            {log.timestamp}: {log.message}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default Logs;

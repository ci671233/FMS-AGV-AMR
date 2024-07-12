import React, { useState, useEffect } from 'react';
import axios from 'axios';

function FactoryManagement() {
  const [pendingFactories, setPendingFactories] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchPendingFactories = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/admin/pending/factory`);
        setPendingFactories(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchPendingFactories();
  }, []);

  return (
    <div>
      <h2>Factory Approval Waiting List</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <ul>
        {pendingFactories.map((factory) => (
          <li key={factory._id}>
            {factory.name} - {factory.location}
            {/* 승인 버튼을 추가하여 공장을 승인할 수 있도록 합니다. */}
          </li>
        ))}
      </ul>
    </div>
  );
}

export default FactoryManagement;

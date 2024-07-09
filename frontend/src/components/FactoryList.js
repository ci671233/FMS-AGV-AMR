import React, { useState, useEffect } from 'react';
import axios from 'axios';
import { Link } from 'react-router-dom';

function FactoryList() {
  const [factories, setFactories] = useState([]);
  const [error, setError] = useState('');

  useEffect(() => {
    const fetchFactories = async () => {
      try {
        const response = await axios.get(`${process.env.REACT_APP_API_URL}/factory`);
        setFactories(response.data);
      } catch (err) {
        setError(err.response.data.message);
      }
    };

    fetchFactories();
  }, []);

  return (
    <div style={{ textAlign: 'center' }}>
      <h2>Factories</h2>
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <ul>
        {factories.map((factory) => (
          <li key={factory._id}>
            <Link to={`/factories/${factory._id}`}>
              {factory.name} - {factory.status === 'approved' ? <span style={{ color: 'green' }}>●</span> : <span style={{ color: 'red' }}>●</span>}
            </Link>
          </li>
        ))}
      </ul>
      <Link to="/create-factory">
        <button style={{ width: '50px', height: '50px', fontSize: '30px', marginTop: '20px' }}>+</button>
      </Link>
    </div>
  );
}

export default FactoryList;

import React, { useState, useContext } from 'react';
import axios from 'axios';
import { useNavigate } from 'react-router-dom';
import { AuthContext } from '../context/AuthContext';

function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const navigate = useNavigate();
  const { setAuthTokens } = useContext(AuthContext);

  const handleLogin = async () => {
    try {
      const response = await axios.post(`${process.env.REACT_APP_API_URL}/account/login`, { email, password });
      const { accessToken } = response.data;
      setAuthTokens(accessToken);
      alert('Login successful');
      navigate('/admin');
    } catch (err) {
      setError('Invalid email or password');
    }
  };

  return (
    <div style={{ textAlign: 'center' }}>
      <h2>Log in</h2>
      <input
        type="text"
        placeholder="Email"
        value={email}
        onChange={(e) => setEmail(e.target.value)}
        style={{ width: '300px', height: '30px', margin: '10px' }}
      />
      <input
        type="password"
        placeholder="PWD"
        value={password}
        onChange={(e) => setPassword(e.target.value)}
        style={{ width: '300px', height: '30px', margin: '10px' }}
      />
      {error && <div style={{ color: 'red' }}>{error}</div>}
      <button onClick={handleLogin} style={{ width: '100px', height: '40px', marginTop: '20px' }}>Login</button>
      <div>
        <a href="/register">register</a>
      </div>
    </div>
  );
}

export default Login;



import React, { useState } from 'react';
import axios from 'axios';

function Login() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');

  const handleLogin = async () => {
    try {
      const response = await axios.post('http://172.30.1.28:8080/api/account/login', {
        email,
        password
      });
      localStorage.setItem('token', response.data.token);
      alert('Login successful');
      console.log(response.data);
    } catch (error) {
      console.error('Login error:', error.response || error); // 전체 응답 객체를 로그로 출력
      alert(`Login failed: ${error.response?.data?.message || error.message}`);
    }
  };

  return (
    <div>
      <h2>Login</h2>
      <input type="text" placeholder="Email" value={email} onChange={(e) => setEmail(e.target.value)} />
      <input type="password" placeholder="Password" value={password} onChange={(e) => setPassword(e.target.value)} />
      <button onClick={handleLogin}>Login</button>
    </div>
  );
}

export default Login;


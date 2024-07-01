import React, { useState } from 'react';
import axios from 'axios';

function Register() {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [phone, setPhone] = useState('');

  const handleRegister = async () => {
    try {
      const response = await axios.post('http://172.30.1.28:8080/api/account/register', {
        email,
        password,
        name,
        phone
      });
      alert('Registration successful. Waiting for approval.');
      console.log(response.data);
    } catch (error) {
      console.error('Registration error:', error.response || error); // 전체 응답 객체를 로그로 출력
      alert(`Registration failed: ${error.response?.data?.message || error.message}`);
    }
  };

  return (
    <div>
      <h2>Register</h2>
      <input type="text" placeholder="Email" value={email} onChange={(e) => setEmail(e.target.value)} />
      <input type="password" placeholder="Password" value={password} onChange={(e) => setPassword(e.target.value)} />
      <input type="text" placeholder="Name" value={name} onChange={(e) => setName(e.target.value)} />
      <input type="text" placeholder="Phone" value={phone} onChange={(e) => setPhone(e.target.value)} />
      <button onClick={handleRegister}>Register</button>
    </div>
  );
}

export default Register;





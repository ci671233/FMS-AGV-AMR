import React, { useState } from 'react';
import axios from 'axios';

function NewAccessToken() {
  const [refreshToken, setRefreshToken] = useState('');

  const handleNewAccessToken = async () => {
    try {
      const response = await axios.post('http://172.30.1.28:8080/api/account/newaccesstoken', {
        refreshToken
      });
      localStorage.setItem('token', response.data.token);
      alert('New access token issued');
      console.log(response.data);
    } catch (error) {
      console.error('New access token error:', error.response || error); // 전체 응답 객체를 로그로 출력
      alert(`Failed to issue new access token: ${error.response?.data?.message || error.message}`);
    }
  };

  return (
    <div>
      <h2>New Access Token</h2>
      <input type="text" placeholder="Refresh Token" value={refreshToken} onChange={(e) => setRefreshToken(e.target.value)} />
      <button onClick={handleNewAccessToken}>Get New Access Token</button>
    </div>
  );
}

export default NewAccessToken;

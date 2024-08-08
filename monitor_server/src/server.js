require('dotenv').config();
const app = require('./app');
const express = require('express');
const WebSocket = require('ws');

const PORT = process.env.PORT;
const WS_PORT = 5050; // WebSocket 서버 포트

app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

// WebSocket 서버 설정
const wss = new WebSocket.Server({ port: WS_PORT });

wss.on('connection', function connection(ws) {
  console.log('WebSocket connected');

  ws.on('message', function message(data) {
    console.log('received: %s', data);
    // 여기서 로봇의 위치 데이터를 처리합니다.
  });

  ws.on('close', () => {
    console.log('WebSocket disconnected');
  });
});

module.exports = { wss };

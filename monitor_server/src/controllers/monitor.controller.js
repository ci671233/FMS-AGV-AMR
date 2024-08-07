const Robot = require('../models/robot.model');
const WebSocket = require('ws');

let clients = [];

exports.monitorUpdates = (ws, req) => {
  clients.push(ws);

  ws.on('message', (message) => {
    // 클라이언트로부터 메시지를 수신할 경우 처리 로직
    console.log('Received message from client:', message);
  });

  ws.on('close', () => {
    clients = clients.filter(client => client !== ws);
  });

  // 주기적으로 로봇 위치 데이터 전송 (예시: 1초마다)
  setInterval(async () => {
    const robotPositions = await getRobotPositions();
    clients.forEach(client => {
      client.send(JSON.stringify(robotPositions));
    });
  }, 1000);
};

// 로봇 위치 데이터를 가져오는 함수
async function getRobotPositions() {
  try {
    const robots = await Robot.find().select('name location');
    return robots;
  } catch (error) {
    console.error('Error fetching robot positions:', error);
    return [];
  }
}



// dotenv 패키지를 로드하여 .env 파일의 환경 변수를 process.env 객체에 로드
require('dotenv').config();

// Express 애플리케이션 로드
const app = require('./app');

// .env 파일에서 환경 변수 불러오기
const PORT = process.env.PORT || 3000;

// 서버를 지정된 포트에서 실행합니다.
const http = require('http');
const server = http.createServer(app);
const socketIO = require('socket.io');
const io = socketIO(server);

// WebSocket 설정
io.on('connection', (socket) => {
  console.log('New WebSocket connection');

  socket.on('signal', (data) => {
    io.emit('signal', data);
  });

  socket.on('disconnect', () => {
    console.log('WebSocket disconnected');
  });
});

server.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

require('dotenv').config();

const app = require('./app');
const http = require('http');
const server = http.createServer(app);
const socketIO = require('socket.io');
const io = socketIO(server, {
  cors: {
    origin: process.env.FRONT_URI,
    methods: ['GET', 'POST'],
    credentials: true
  }
});

const PORT = process.env.PORT;

io.on('connection', (socket) => {
  console.log('New WebSocket connection');

  socket.on('signal', (data) => {
    io.emit('signal', data);
  });

  socket.on('key_press', (data) => {
    io.emit('key_press', data);
  });

  socket.on('disconnect', () => {
    console.log('WebSocket disconnected');
  });
});

server.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

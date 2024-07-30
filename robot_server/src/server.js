require('dotenv').config();

const app = require('./app');
const http = require('http');
const server = http.createServer(app);
const socketIO = require('socket.io');
const io = socketIO(server);

const PORT = process.env.PORT || 3000;

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

require('dotenv').config();
const app = require('./app');

const PORT = process.env.PORT || 5558;

const server = app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});

const WebSocket = require('ws');
const wss = new WebSocket.Server({ server });

let clients = [];

wss.on('connection', function connection(ws) {
  clients.push(ws);
  console.log('WebSocket connected');

  ws.on('message', (message) => {
    console.log('Received message from client:', message);
  });

  ws.on('close', () => {
    clients = clients.filter(client => client !== ws);
    console.log('WebSocket disconnected');
  });
});

function broadcastPosition(position) {
  clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify(position));
    }
  });
}

module.exports = {
  broadcastPosition
};

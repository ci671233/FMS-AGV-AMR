const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const { spawn } = require('child_process');

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
    cors: {
        origin: '*',
        methods: ['GET', 'POST'],
        credentials: true
    }
});

app.use(express.static('public'));

io.on('connection', (socket) => {
    console.log('New client connected');

    const ffmpeg = spawn('ffmpeg', [
        '-i', 'http://172.30.1.40:8080/stream?topic=/camera/color/image_raw',
        '-f', 'mjpeg',
        'pipe:1'
    ]);

    ffmpeg.stdout.on('data', (data) => {
        socket.emit('video', data.toString('base64'));
    });

    ffmpeg.stderr.on('data', (data) => {
        console.error(`ffmpeg stderr: ${data}`);
    });

    ffmpeg.on('close', (code) => {
        console.log(`ffmpeg process exited with code ${code}`);
    });

    socket.on('disconnect', () => {
        console.log('Client disconnected');
        ffmpeg.kill('SIGINT');
    });
});

const PORT = process.env.PORT || 7001;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));

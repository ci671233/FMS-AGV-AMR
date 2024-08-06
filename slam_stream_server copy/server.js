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

let ffmpeg;
let slamStream;

const startFFmpeg = () => {
    console.log('Starting ffmpeg stream...');
    ffmpeg = spawn('ffmpeg', [
        '-i', 'http://localhost:8080/stream?topic=/map', // Replace with your RViz topic stream
        '-f', 'mpegts',
        '-codec:v', 'mpeg1video',
        '-s', '640x480',
        '-b:v', '800k',
        '-r', '30',
        'pipe:1'
    ]);

    slamStream = ffmpeg.stdout;

    ffmpeg.stderr.on('data', (data) => {
        console.error(`ffmpeg stderr: ${data}`);
    });

    ffmpeg.on('close', (code) => {
        console.log(`ffmpeg process exited with code ${code}`);
    });

    return slamStream;
};

io.on('connection', (socket) => {
    console.log('New client connected');

    if (!slamStream) {
        slamStream = startFFmpeg();
    }

    slamStream.on('data', (data) => {
        socket.emit('slam-stream', data);
    });

    socket.on('disconnect', () => {
        console.log('Client disconnected');
    });
});

const PORT = process.env.PORT || 7002;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));

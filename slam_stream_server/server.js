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

    // 정확한 rviz 창의 위치와 크기
    const x = 72; // Absolute upper-left X 값
    const y = 64; // Absolute upper-left Y 값
    const width = 1848; // Width 값
    const height = 1016; // Height 값

    const targetWidth = 640;
    const targetHeight = 480;

    const ffmpeg = spawn('ffmpeg', [
        '-f', 'x11grab',
        '-r', '30',
        '-s', `${width}x${height}`,
        '-i', `:0.0+${x},${y}`,
        '-vf', `scale=${targetWidth}:${targetHeight}`,
        '-b:v', '1M', // 비트레이트 설정
        '-preset', 'veryfast',
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

const PORT = process.env.PORT || 7002;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));

const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const wrtc = require('wrtc');
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
let peerConnection;
let videoStream = new wrtc.MediaStream();
let videoTrack;

const startFFmpeg = () => {
    console.log('Starting ffmpeg stream...');
    ffmpeg = spawn('ffmpeg', [
        '-i', 'http://172.30.1.40:8080/stream?topic=/camera/color/image_raw',
        '-f', 'mpegts',
        '-codec:v', 'mpeg1video',
        '-s', '640x480',
        '-b:v', '800k',
        '-r', '30',
        'pipe:1'
    ]);

    ffmpeg.stdout.on('data', (data) => {
        if (videoTrack) {
            videoTrack.write(data);
        }
    });

    ffmpeg.stderr.on('data', (data) => {
        console.error(`ffmpeg stderr: ${data}`);
    });

    ffmpeg.on('close', (code) => {
        console.log(`ffmpeg process exited with code ${code}`);
    });
};

io.on('connection', (socket) => {
    console.log('New client connected');

    socket.on('join', async () => {
        if (!peerConnection) {
            peerConnection = new wrtc.RTCPeerConnection();

            peerConnection.onicecandidate = (event) => {
                if (event.candidate) {
                    socket.emit('candidate', event.candidate);
                }
            };

            peerConnection.oniceconnectionstatechange = () => {
                console.log('ICE Connection State Change:', peerConnection.iceConnectionState);
            };

            peerConnection.onconnectionstatechange = () => {
                console.log('Connection State Change:', peerConnection.connectionState);
            };

            const videoTrack = new wrtc.MediaStreamTrack();
            videoStream.addTrack(videoTrack);

            peerConnection.addStream(videoStream);

            startFFmpeg();
        }
    });

    socket.on('signal', async (data) => {
        if (!peerConnection) {
            return;
        }
        if (data.type === 'offer') {
            await peerConnection.setRemoteDescription(new wrtc.RTCSessionDescription(data));
            const answer = await peerConnection.createAnswer();
            await peerConnection.setLocalDescription(answer);
            socket.emit('signal', peerConnection.localDescription);
        } else if (data.type === 'answer') {
            await peerConnection.setRemoteDescription(new wrtc.RTCSessionDescription(data));
        }
    });

    socket.on('candidate', async (candidate) => {
        if (!peerConnection) {
            return;
        }
        try {
            await peerConnection.addIceCandidate(new wrtc.RTCIceCandidate(candidate));
        } catch (error) {
            console.error('Error adding received ICE candidate', error);
        }
    });

    socket.on('disconnect', () => {
        console.log('Client disconnected');
        if (peerConnection) {
            peerConnection.close();
            peerConnection = null;
        }
        if (ffmpeg) {
            ffmpeg.kill('SIGINT');
        }
    });
});

const PORT = process.env.PORT || 7001;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));

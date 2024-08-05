// server.js
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const wrtc = require('wrtc');
const { spawn } = require('child_process');
const { Readable } = require('stream');

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

    const videoStream = new Readable({
        read() {}
    });

    ffmpeg.stdout.on('data', (data) => {
        videoStream.push(data);
    });

    ffmpeg.stderr.on('data', (data) => {
        console.error(`ffmpeg stderr: ${data}`);
    });

    ffmpeg.on('close', (code) => {
        console.log(`ffmpeg process exited with code ${code}`);
        videoStream.push(null);
    });

    return videoStream;
};

io.on('connection', (socket) => {
    console.log('New client connected');

    socket.on('join', async () => {
        const peerConnection = new wrtc.RTCPeerConnection();

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

        const videoStream = startFFmpeg();

        videoStream.on('data', (data) => {
            if (peerConnection.connectionState === 'connected') {
                const videoTrack = peerConnection.getSenders().find(sender => sender.track.kind === 'video').track;
                videoTrack.write(data);
            }
        });

        const offer = await peerConnection.createOffer();
        await peerConnection.setLocalDescription(offer);

        socket.emit('signal', peerConnection.localDescription);

        socket.on('signal', async (data) => {
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
            try {
                await peerConnection.addIceCandidate(new wrtc.RTCIceCandidate(candidate));
            } catch (error) {
                console.error('Error adding received ICE candidate', error);
            }
        });

        socket.on('disconnect', () => {
            console.log('Client disconnected');
            peerConnection.close();
        });
    });
});

const PORT = process.env.PORT || 7001;
server.listen(PORT, () => console.log(`Server running on port ${PORT}`));



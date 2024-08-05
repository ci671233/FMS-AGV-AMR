import React, { useEffect, useRef, useState } from 'react';
import SimplePeer from 'simple-peer';
import io from 'socket.io-client';

const WebcamStream = ({ selectedRobot }) => {
    const videoRef = useRef(null);
    const [peer, setPeer] = useState(null);
    const socketRef = useRef(null);

    useEffect(() => {
        if (selectedRobot) {
            socketRef.current = io('http://172.30.1.40:7001'); // WebRTC 서버 주소

            const peer = new SimplePeer({ initiator: true, trickle: false });

            peer.on('signal', data => {
                if (socketRef.current) {
                    socketRef.current.emit('signal', data);
                }
            });

            peer.on('stream', stream => {
                if (videoRef.current) {
                    videoRef.current.srcObject = stream;
                }
            });

            peer.on('error', (err) => {
                console.error('Peer connection error:', err);
            });

            socketRef.current.on('signal', signal => {
                if (peer.destroyed) {
                    console.warn('Peer is destroyed, cannot signal');
                    return;
                }
                peer.signal(signal);
            });

            socketRef.current.on('candidate', candidate => {
                if (peer.destroyed) {
                    console.warn('Peer is destroyed, cannot add candidate');
                    return;
                }
                peer.signal(candidate);
            });

            socketRef.current.emit('join');

            setPeer(peer);

            return () => {
                peer.destroy();
                socketRef.current.disconnect();
            };
        }
    }, [selectedRobot]);

    return (
        <div>
            <video ref={videoRef} autoPlay playsInline />
        </div>
    );
};

export default WebcamStream;



import React, { useEffect, useRef } from 'react';
import io from 'socket.io-client';

const SlamStream = ({ selectedRobot }) => {
    const videoRef = useRef(null);

    useEffect(() => {
        if (selectedRobot) {
            const socket = io('http://172.30.1.40:7002');
            socket.on('slam-stream', (data) => {
                const videoElement = videoRef.current;
                const blob = new Blob([data], { type: 'video/mp2t' });
                videoElement.src = URL.createObjectURL(blob);
                videoElement.play();
            });

            return () => {
                socket.disconnect();
            };
        }
    }, [selectedRobot]);

    return (
        <div>
            <video ref={videoRef} autoPlay playsInline style={{ width: '100%' }} />
        </div>
    );
};

export default SlamStream;

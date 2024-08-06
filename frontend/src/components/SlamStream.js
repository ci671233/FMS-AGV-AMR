import React, { useEffect, useRef } from 'react';
import io from 'socket.io-client';

const SlamStream = () => {
    const videoRef = useRef();

    useEffect(() => {
        const socket = io('http://172.30.1.40:7002');

        socket.on('video', (data) => {
            if (videoRef.current) {
                videoRef.current.src = `data:image/jpeg;base64,${data}`;
            }
        });

        return () => {
            socket.disconnect();
        };
    }, []);

    return (
        <div>
            <img ref={videoRef} alt="SLAM Stream" />
        </div>
    );
};

export default SlamStream;




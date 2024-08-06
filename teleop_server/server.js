const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const rosnodejs = require('rosnodejs');
const geometry_msgs = rosnodejs.require('geometry_msgs').msg;

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
    cors: {
        origin: '*',
        methods: ['GET', 'POST'],
        credentials: true
    }
});

rosnodejs.initNode('/teleop_node')
    .then((rosNode) => {
        const pub = rosNode.advertise('/cmd_vel', geometry_msgs.Twist);

        io.on('connection', (socket) => {
            console.log('New client connected');

            socket.on('teleop', (data) => {
                const twist = new geometry_msgs.Twist();
                twist.linear.x = data.command.linear;
                twist.angular.z = data.command.angular;
                pub.publish(twist);
            });

            socket.on('disconnect', () => {
                console.log('Client disconnected');
            });
        });

        const PORT = process.env.PORT || 7003;
        server.listen(PORT, () => console.log(`Server running on port ${PORT}`));
    });

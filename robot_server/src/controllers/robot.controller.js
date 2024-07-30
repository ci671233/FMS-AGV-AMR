const mongoose = require('mongoose');
const axios = require('axios');
const Robot = require('../models/robot.model');
const rosnodejs = require('rosnodejs');
const ROSLIB = require('roslib');
const WebSocket = require('ws');
const { exec } = require('child_process');

// ROS 노드 초기화
rosnodejs.initNode('/web_server_node')
  .then(() => {
    console.log('ROS node initialized');
  })
  .catch((err) => {
    console.error('Failed to initialize ROS node:', err);
  });

exports.getRobots = async (req, res) => {
  try {
    const robots = await Robot.find();
    res.json(robots);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch robots');
  }
};

exports.registerRobot = async (req, res) => {
  const { name, ip, model } = req.body;
  const robot = new Robot({ name, ip, model });
  try {
    await robot.save();
    res.send('Robot registered');
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to register robot');
  }
};

exports.sendCommand = async (req, res) => {
  const { robot_id, command } = req.body;
  const { REMOTE_PC_USER, REMOTE_PC_IP } = process.env;

  try {
    const robot = await Robot.findById(robot_id);
    if (!robot) {
      return res.status(404).send('Robot not found');
    }

    let scriptPath;
    if (command === 'slam') {
      scriptPath = '~/scripts/start_slam.sh';
    } else {
      return res.status(400).send('Unknown command');
    }

    exec(`ssh ${REMOTE_PC_USER}@${REMOTE_PC_IP} "bash ${scriptPath}"`, (error, stdout, stderr) => {
      if (error) {
        console.error(`exec error: ${error}`);
        return res.status(500).send(`Failed to start ${command}`);
      }
      console.log(`stdout: ${stdout}`);
      console.error(`stderr: ${stderr}`);
      res.send(`${command} started`);
    });
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to send command');
  }
};

// WebSocket 서버 설정
const wss = new WebSocket.Server({ port: process.env.WS_PORT });

wss.on('connection', (ws) => {
  ws.on('message', async (message) => {
    const { robot_id, velocity } = JSON.parse(message);
    try {
      const robot = await Robot.findById(robot_id);
      if (!robot) {
        return;
      }

      const nh = rosnodejs.nh;
      const cmdVel = new ROSLIB.Topic({
        ros: nh,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
      });

      const twist = new ROSLIB.Message({
        linear: { x: velocity.linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: velocity.angular }
      });

      cmdVel.publish(twist);
    } catch (error) {
      console.error(`WebSocket error: ${error}`);
    }
  });
});



const mongoose = require('mongoose');
const rosnodejs = require('rosnodejs');
const Robot = require('../models/robot.model');
const WebSocket = require('ws');
const ROSLIB = require('roslib');
const WS_PORT = process.env.WS_PORT;

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
  try {
    const robot = await Robot.findById(robot_id);
    if (!robot) {
      return res.status(404).send('Robot not found');
    }

    if (command === 'slam') {
      const nh = rosnodejs.nh;
      const serviceClient = nh.serviceClient('/start_slam', 'std_srvs/Trigger');
      await serviceClient.waitForService();
      const resp = await serviceClient.call({});
      console.log('SLAM started: ', resp);
      res.send(resp);
    } else {
      res.status(400).send('Unknown command');
    }
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to send command');
  }
};

// WebSocket 서버 설정
const wss = new WebSocket.Server({ port: WS_PORT });

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



// exports.sendCommand = async (req, res) => {
//   const { robot_id, command } = req.body;
//   try {
//     const robot = await Robot.findById(robot_id);
//     const response = await axios.post(`${TURTLEBOT_SERVER_URL}/robot/send_command`, { robot_ip: robot.ip, command });
//     res.send(response.data);
//   } catch (error) {
//     console.error(`Error: ${error}`);
//     res.status(500).send('Failed to send command');
//   }
// };
  
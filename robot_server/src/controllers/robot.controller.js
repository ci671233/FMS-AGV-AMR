const mongoose = require('mongoose');
const axios = require('axios');
const Robot = require('../models/robot.model');
// const conn = mongoose.connection;
// const jwt = require('jsonwebtoken');
const TURTLEBOT_SERVER_URL = process.env.TURTLEBOT_SERVER_URL;

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
    const response = await axios.post(`${TURTLEBOT_SERVER_URL}/send_command`, { robot_ip: robot.ip, command });
    res.send(response.data);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to send command');
  }
};
  
const mongoose = require('mongoose');
const axios = require('axios');
const Robot = require('../models/robot.model');
const rosnodejs = require('rosnodejs');

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


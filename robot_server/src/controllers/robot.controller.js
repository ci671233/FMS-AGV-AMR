const mongoose = require('mongoose');
const Robot = require('../models/robot.model');
const axios = require('axios');

// 자신이 보유한 로봇 조회
exports.getRobots = async (req, res) => {
  try {
    const robots = await Robot.find({ userId: req.user.id });
    res.json(robots);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch robots');
  }
};

// 로봇 등록
exports.registerRobot = async (req, res) => {
  try {
    const { name, ip, model } = req.body;

    const robot = new Robot({
      name,
      ip,
      model,
      userId: req.user.id
    });

    await robot.save();
    res.status(201).json({ message: 'Robot registered successfully', robot });
  } catch (error) {
    console.error('Error registering robot:', error);
    res.status(500).json({ message: 'Error registering robot', error: error.message });
  }
};

// 로봇 업데이트
exports.updateRobot = async (req, res) => {
  try {
    console.log('Request user:', req.user); // 사용자 정보 로그 추가
    console.log('Request body:', req.body); // 요청 본문 로그 추가

    const { name, ip, model } = req.body;
    const { id } = req.params;
    console.log('Robot ID:', id); // robotId 로그 추가
    console.log('User ID:', req.user.id); // userId 로그 추가

    const robot = await Robot.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { name, ip, model },
      { new: true, runValidators: true } // runValidators 옵션 추가
    );

    if (!robot) {
      return res.status(404).json({ message: 'Robot not found or not authorized' });
    }

    res.status(200).json({ message: 'Robot updated successfully', robot });
  } catch (error) {
    console.error('Error updating robot:', error);
    res.status(500).json({ message: 'Error updating robot', error: error.message });
  }
};

// 로봇에게 맵 전송
exports.sendMapToRobot = async (req, res) => {
  try {
    const { robotId } = req.body;

    // 로봇 정보 조회
    const robot = await Robot.findById(robotId);
    if (!robot) {
      return res.status(404).json({ message: 'Robot not found' });
    }

    // 선택된 맵 정보 조회
    const response = await axios.get(`http://172.30.1.40:5557/map/monitored`, {
      headers: { Authorization: req.headers.authorization }
    });
    const map = response.data;

    // 로봇에게 맵 URL 전송
    const mapUrl = `http://172.30.1.40:5557/map/file/${map.FileId}`;
    await axios.post(`http://${robot.ip}:5000/receive_map`, { mapUrl });
    res.status(200).json({ message: 'Map sent to robot successfully' });
  } catch (error) {
    console.error('Error sending map to robot:', error);
    res.status(500).json({ message: 'Error sending map to robot', error: error.message });
  }
};
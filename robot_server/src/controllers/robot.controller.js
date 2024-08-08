const mongoose = require('mongoose');
const Robot = require('../models/robot.model');
const axios = require('axios');
const { GridFSBucket } = require('mongodb');
const WebSocket = require('ws');
const Map = require('../models/map.model');
const rosnodejs = require('rosnodejs');

let gfs;
mongoose.connection.once('open', () => {
  gfs = new GridFSBucket(mongoose.connection.db, { bucketName: 'maps' });
});


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
exports.sendMapToRobots = async (req, res) => {
  try {
    // ROS 노드 초기화
    await rosnodejs.initNode('/map_publisher_node');
    const nh = rosnodejs.nh;

    const { userId } = req.user;

    // 선택된 맵 정보 조회
    const monitoredMapResponse = await axios.get('http://172.30.1.40:5557/map/monitored', {
      headers: { Authorization: req.headers.authorization }
    });

    const monitoredMap = monitoredMapResponse.data;

    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }

    console.log(`Monitored map ID: ${monitoredMap._id}`);

    // 맵 파일 스트림 생성
    const response = await axios.get(`http://172.30.1.40:5557/map/file/${monitoredMap.FileId}`, {
      responseType: 'arraybuffer',
      headers: { Authorization: req.headers.authorization }
    });

    const mapData = Buffer.from(response.data, 'binary');
    console.log(`Map data size: ${mapData.length}`);

    // ROS 메시지 생성
    const std_msgs = rosnodejs.require('std_msgs').msg;
    const mapTopic = nh.advertise('/map_topic', std_msgs.String);
    const mapMsg = new std_msgs.String({
      data: mapData.toString('base64')
    });

    // ROS 토픽으로 맵 데이터 퍼블리시
    mapTopic.publish(mapMsg);
    console.log('Map data published to /map_topic');
    
    res.status(200).json({ message: 'Map sent to robots successfully' });
  } catch (error) {
    console.error('Error sending map to robots:', error);
    res.status(500).json({ message: 'Error sending map to robots', error: error.message });
  }
};
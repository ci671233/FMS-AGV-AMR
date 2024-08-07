const mongoose = require('mongoose');
const { GridFSBucket } = require('mongodb');
const Map = require('../models/map.model');
const multer = require('multer');

// GridFS 초기화
let gfs;
mongoose.connection.once('open', () => {
  gfs = new GridFSBucket(mongoose.connection.db, { bucketName: 'maps' });
});

// Multer 설정
const storage = multer.memoryStorage();
const upload = multer({ storage });

// 자신이 보유한 맵 조회
exports.getMaps = async (req, res) => {
  try {
    const maps = await Map.find({ userId: req.user.id });
    res.json(maps);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch maps');
  }
};

// 맵 업로드
exports.uploadMap = [
  upload.single('file'),
  async (req, res) => {
    try {
      console.log('Request user:', req.user); // 사용자 정보 로그 추가
      console.log('Request body:', req.body);
      console.log('Request file:', req.file);

      const { name, description } = req.body;
      const file = req.file;

      if (!file) {
        return res.status(400).json({ message: 'File is required' });
      }

      const uploadStream = gfs.openUploadStream(file.originalname, {
        contentType: file.mimetype
      });

      let fileId;

      const uploadPromise = new Promise((resolve, reject) => {
        uploadStream.on('finish', (uploadedFile) => {
          fileId = uploadedFile._id;
          resolve();
        });
        uploadStream.on('error', reject);
        uploadStream.end(file.buffer);
      });

      await uploadPromise;

      const map = new Map({
        name,
        description,
        FileId: fileId,
        userId: req.user.id
      });

      await map.save();
      res.status(201).json({ message: 'Map uploaded successfully', map });
    } catch (error) {
      console.error('Error uploading map:', error);
      res.status(500).json({ message: 'Error uploading map', error: error.message });
    }
  }
];

// 맵 업데이트
exports.updateMap = async (req, res) => {
  try {
    console.log('Request user:', req.user); // 사용자 정보 로그 추가
    console.log('Request body:', req.body); // 요청 본문 로그 추가

    const { name, description, isMonitored } = req.body;
    const { id } = req.params;
    console.log('Map ID:', id); // mapId 로그 추가
    console.log('User ID:', req.user.id); // userId 로그 추가

    // 모든 맵의 isMonitored 필드를 false로 설정
    if (isMonitored) {
      await Map.updateMany({ userId: req.user.id }, { isMonitored: false });
    }

    // 선택된 맵의 isMonitored 필드를 true로 설정
    const map = await Map.findOneAndUpdate(
      { _id: id, userId: req.user.id },
      { name, description, isMonitored },
      { new: true, runValidators: true } // runValidators 옵션 추가
    );

    if (!map) {
      return res.status(404).json({ message: 'Map not found or not authorized' });
    }

    res.status(200).json({ message: 'Map updated successfully', map });
  } catch (error) {
    console.error('Error updating map:', error);
    res.status(500).json({ message: 'Error updating map', error: error.message });
  }
};

// 선택된 맵 조회
exports.getMonitoredMap = async (req, res) => {
  try {
    const monitoredMap = await Map.findOne({ userId: req.user.id, isMonitored: true });
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }
    res.status(200).json(monitoredMap);
  } catch (error) {
    console.error('Error fetching monitored map:', error);
    res.status(500).json({ message: 'Error fetching monitored map', error: error.message });
  }
};

// 선택된 맵 파일 다운로드
exports.getMonitoredMapFile = async (req, res) => {
  try {
    const monitoredMap = await Map.findOne({ userId: req.user.id, isMonitored: true });
    if (!monitoredMap) {
      return res.status(404).json({ message: 'No monitored map found' });
    }
    
    const fileId = monitoredMap.FileId;
    const downloadStream = gfs.openDownloadStream(fileId);

    downloadStream.on('error', (error) => {
      console.error('Error downloading map file:', error);
      res.status(500).json({ message: 'Error downloading map file', error: error.message });
    });

    downloadStream.pipe(res);
  } catch (error) {
    console.error('Error fetching monitored map file:', error);
    res.status(500).json({ message: 'Error fetching monitored map file', error: error.message });
  }
};

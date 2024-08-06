const mongoose = require('mongoose');
const axios = require('axios');
const jwt = require('jsonwebtoken');
const Map = require('../models/map.model');
const multer = require('multer'); 
const { GridFSBucket } = require('mongodb');

// GridFS 초기화
let gfs;
mongoose.connection.once('open', () => {
  gfs = new GridFSBucket(mongoose.connection.db, { bucketName: 'maps' });
});

// Multer 설정
const storage = multer.memoryStorage();
const upload = multer({ storage });

exports.getMaps = async (req, res) => {
  try {
    const maps = await Map.find();
    res.json(maps);
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to fetch maps');
  }
};

exports.saveMap = async (req, res) => {
  const { name, data } = req.body;
  const map = new Map({ name, data });
  try {
    await map.save();
    res.send('Map saved');
  } catch (error) {
    console.error(`Error: ${error}`);
    res.status(500).send('Failed to save map');
  }
};

// 맵 업로드
exports.uploadMap = [
  upload.single('file'),
  async (req, res) => {
    try {
      const { name, description } = req.body;
      const file = req.file;

      if (!file) {
        return res.status(400).json({ message: 'File is required' });
      }

      console.log('Uploading file:', file);
      console.log('Map name:', name);
      console.log('Map description:', description);

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
        FileId: fileId
      });

      await map.save();
      console.log('Map saved to database:', map);
      res.status(201).json({ message: 'Map uploaded successfully', map });
    } catch (error) {
      console.error('Error uploading map:', error);
      res.status(500).json({ message: 'Error uploading map', error: error.message });
    }
  }
];

// exports.getMaps = async (req, res) => {
//   try {
//     const maps = await Map.find();
//     res.json(maps);
//   } catch (error) {
//     res.status(500).json({ message: 'Error fetching maps', error: error.message });
//   }
// };

// exports.getMapById = async (req, res) => {
//   try {
//     const map = await Map.findById(req.params.id);
//     if (!map) {
//       return res.status(404).json({ message: 'Map not found' });
//     }
//     res.json(map);
//   } catch (error) {
//     res.status(500).json({ message: 'Error fetching map', error: error.message });
//   }
// };


// exports.getPngMapById = async (req, res) => {
//   try {
//     const mapId = req.params.id;
//     const map = await Map.findById(mapId);
//     if (!map) {
//       return res.status(404).json({ message: 'Map not found' });
//     }

//     const downloadStream = gfs.openDownloadStream(mongoose.Types.ObjectId(map.pngFileId));

//     res.set('Content-Type', 'image/png');
//     downloadStream.pipe(res);

//     downloadStream.on('error', (error) => {
//       console.error('Error reading map file:', error); // 추가된 로그
//       res.status(500).json({ message: 'Error reading map file', error: error.message });
//     });
//   } catch (error) {
//     console.error('Error fetching map image:', error); // 추가된 로그
//     res.status(500).json({ message: 'Error fetching map image', error: error.message });
//   }
// };

// // selectMapForMonitoring 함수
// exports.selectMapForMonitoring = async (req, res) => {
//   try {
//     const { mapId, userId } = req.body;
//     if (!userId) {
//       return res.status(400).json({ message: 'User ID is missing in request.' });
//     }

//     const token = req.headers.authorization.split(' ')[1];
//     const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
//     if (decoded.id !== userId) {
//       return res.status(401).json({ message: 'Unauthorized user.' });
//     }

//     // account_server에서 사용자 정보 확인 및 업데이트
//     const response = await axios.put(`http://localhost:5555/account/user/${userId}`, { selectedMapId: mapId }, {
//       headers: { Authorization: `Bearer ${token}` }
//     });

//     if (response.status !== 200) {
//       return res.status(response.status).json({ message: 'Error updating user with selected map.' });
//     }

//     res.status(200).json({ message: 'Map selected for monitoring successfully.' });
//   } catch (error) {
//     console.error('Error selecting map for monitoring:', error);
//     res.status(500).json({ message: 'Internal server error' });
//   }
// };




// exports.getSelectedMapImage = async (req, res) => {
//   try {
//     const userId = req.params.id;
//     const token = req.headers.authorization.split(' ')[1];
//     const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
//     if (decoded.id !== userId) {
//       return res.status(401).json({ message: 'Unauthorized user.' });
//     }

//     // account_server에서 사용자 정보 확인
//     const response = await axios.get(`http://localhost:5555/account/user/${userId}`, {
//       headers: { Authorization: `Bearer ${token}` }
//     });

//     if (response.status !== 200) {
//       return res.status(response.status).json({ message: 'Error fetching user data.' });
//     }

//     const user = response.data;
//     const MapId = user.selectedMapId;

//     if (!MapId) {
//       return res.status(404).json({ message: 'No map selected for monitoring.' });
//     }

//     // Map 정보 가져오기
//     const map = await Map.findById(MapId);
//     if (!map) {
//       return res.status(404).json({ message: 'Map not found' });
//     }

//     // Map 이미지 가져오기
//     const downloadStream = gfs.openDownloadStream(mongoose.Types.ObjectId(map.pngFileId));

//     let imageData = '';
//     downloadStream.on('data', (chunk) => {
//       imageData += chunk.toString('base64');
//     });

//     downloadStream.on('end', () => {
//       res.status(200).json({ mapImage: `data:image/png;base64,${imageData}` });
//     });

//     downloadStream.on('error', (error) => {
//       console.error('Error reading map file:', error);
//       res.status(500).json({ message: 'Error reading map file', error: error.message });
//     });

//   } catch (error) {
//     console.error('Error fetching selected map image:', error);
//     res.status(500).json({ message: 'Error fetching selected map image', error: error.message });
//   }
// };
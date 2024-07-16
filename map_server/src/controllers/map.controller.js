const mongoose = require('mongoose');
const { GridFSBucket } = require('mongodb');
const sharp = require('sharp');
const Map = require('../models/map.model');
const conn = mongoose.connection;

let gfs;
conn.once('open', () => {
  gfs = new GridFSBucket(conn.db, { bucketName: 'maps' });
});

exports.getMaps = async (req, res) => {
  try {
    const maps = await Map.find();
    res.json(maps);
  } catch (error) {
    res.status(500).json({ message: 'Error fetching maps', error: error.message });
  }
};

exports.getMapById = async (req, res) => {
  try {
    const map = await Map.findById(req.params.id);
    if (!map) {
      return res.status(404).json({ message: 'Map not found' });
    }
    res.json(map);
  } catch (error) {
    res.status(500).json({ message: 'Error fetching map', error: error.message });
  }
};

exports.uploadMap = async (req, res) => {
  try {
    const { name, description } = req.body;
    const file = req.file;

    console.log('Uploading file:', file);
    console.log('Map name:', name);
    console.log('Map description:', description);

    if (!file) {
      return res.status(400).json({ message: 'No file uploaded' });
    }

    const uploadStream = gfs.openUploadStream(file.originalname, {
      contentType: file.mimetype
    });

    uploadStream.on('finish', async (uploadedFile) => {
      try {
        console.log('File uploaded to GridFS:', uploadedFile);

        const map = new Map({
          name,
          description,
          fileId: uploadedFile._id,
          fileName: file.originalname  // 파일 이름을 저장
        });

        await map.save();
        console.log('Map saved to database:', map);
        res.status(201).json({ message: 'Map uploaded successfully', map });
      } catch (error) {
        console.error('Error saving map:', error);
        res.status(500).json({ message: 'Error saving map', error: error.message });
      }
    });

    uploadStream.on('error', (error) => {
      console.error('Error uploading file to GridFS:', error);
      res.status(500).json({ message: 'Error uploading file to GridFS', error: error.message });
    });

    uploadStream.end(file.buffer);
  } catch (error) {
    console.error('Error uploading map:', error);
    res.status(500).json({ message: 'Error uploading map', error: error.message });
  }
};

exports.getSelectedMap = async (req, res) => {
  try {
    const selectedMapId = req.user.selectedMapId;
    const map = await Map.findById(selectedMapId);
    if (!map) {
      return res.status(404).json({ message: 'Selected map not found' });
    }

    const downloadStream = gfs.openDownloadStream(mongoose.Types.ObjectId(map.fileId));

    let data = [];
    downloadStream.on('data', (chunk) => {
      data.push(chunk);
    });

    downloadStream.on('end', async () => {
      const fileBuffer = Buffer.concat(data);

      // Convert .pgm to .png using sharp
      const pngBuffer = await sharp(fileBuffer).png().toBuffer();

      res.json({ mapId: selectedMapId, mapImage: pngBuffer.toString('base64'), mapDescription: map.description });
    });

    downloadStream.on('error', (error) => {
      res.status(500).json({ message: 'Error reading map file', error: error.message });
    });
  } catch (error) {
    res.status(500).json({ message: 'Error fetching selected map', error: error.message });
  }
};

exports.downloadMap = async (req, res) => {
  try {
    const mapId = req.params.id;
    const map = await Map.findById(mapId);
    if (!map) {
      return res.status(404).json({ message: 'Map not found' });
    }

    console.log('Downloading map:', map);

    const downloadStream = gfs.openDownloadStream(mongoose.Types.ObjectId(map.fileId));

    downloadStream.on('error', (error) => {
      res.status(500).json({ message: 'Error reading map file', error: error.message });
    });

    const originalExtension = map.fileName.split('.').pop();
    const downloadFileName = `${map.name}.${originalExtension}`;

    console.log('Original filename:', map.fileName);
    console.log('Download filename:', downloadFileName);

    res.set('Content-Type', map.contentType || 'application/octet-stream');
    res.set('Content-Disposition', `attachment; filename="${downloadFileName}"`); // 파일 이름과 확장자 설정
    downloadStream.pipe(res);
  } catch (error) {
    res.status(500).json({ message: 'Error downloading map', error: error.message });
  }
};




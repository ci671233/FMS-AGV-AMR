const mongoose = require('mongoose');
const { GridFSBucket } = require('mongodb');
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

    if (!file) {
      return res.status(400).json({ message: 'No file uploaded' });
    }

    const uploadStream = gfs.openUploadStream(file.originalname, {
      contentType: file.mimetype
    });

    uploadStream.on('finish', async (uploadedFile) => {
      try {
        const map = new Map({
          name,
          description,
          fileId: uploadedFile._id
        });

        await map.save();
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

    downloadStream.on('end', () => {
      const fileBuffer = Buffer.concat(data);
      res.json({ mapId: selectedMapId, mapImage: fileBuffer.toString('base64'), mapDescription: map.description });
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

    const downloadStream = gfs.openDownloadStream(mongoose.Types.ObjectId(map.fileId));

    downloadStream.on('error', (error) => {
      res.status(500).json({ message: 'Error reading map file', error: error.message });
    });

    res.set('Content-Type', 'application/octet-stream');
    res.set('Content-Disposition', `attachment; filename=${map.name}`);
    downloadStream.pipe(res);
  } catch (error) {
    res.status(500).json({ message: 'Error downloading map', error: error.message });
  }
};


  




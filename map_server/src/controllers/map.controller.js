const mongoose = require('mongoose');
const { GridFSBucket } = require('mongodb');
const Map = require('../models/map.model');
const convertPgmToPng = require('../utils/convertPgmToPng');
const conn = mongoose.connection;

let gfs;
conn.once('open', () => {
  gfs = new GridFSBucket(conn.db, { bucketName: 'maps' });
});

exports.uploadMap = async (req, res) => {
  try {
    const { name, description } = req.body;
    const pgmFile = req.files.pgm[0];
    const yamlFile = req.files.yaml[0];

    console.log('Uploading files:', pgmFile, yamlFile);
    console.log('Map name:', name);
    console.log('Map description:', description);

    if (!pgmFile || !yamlFile) {
      return res.status(400).json({ message: 'Both .pgm and .yaml files are required' });
    }

    const uploadPgmStream = gfs.openUploadStream(pgmFile.originalname, {
      contentType: pgmFile.mimetype
    });

    const uploadYamlStream = gfs.openUploadStream(yamlFile.originalname, {
      contentType: yamlFile.mimetype
    });

    let pgmFileId, yamlFileId, pngFileId;

    const uploadPgmPromise = new Promise((resolve, reject) => {
      uploadPgmStream.on('finish', (uploadedFile) => {
        pgmFileId = uploadedFile._id;
        resolve();
      });
      uploadPgmStream.on('error', reject);
      uploadPgmStream.end(pgmFile.buffer);
    });

    const uploadYamlPromise = new Promise((resolve, reject) => {
      uploadYamlStream.on('finish', (uploadedFile) => {
        yamlFileId = uploadedFile._id;
        resolve();
      });
      uploadYamlStream.on('error', reject);
      uploadYamlStream.end(yamlFile.buffer);
    });

    await Promise.all([uploadPgmPromise, uploadYamlPromise]);

    // Convert PGM to PNG and upload to GridFS
    const pngBuffer = await convertPgmToPng(pgmFile.buffer);
    const uploadPngStream = gfs.openUploadStream(pgmFile.originalname.replace('.pgm', '.png'), {
      contentType: 'image/png'
    });

    const uploadPngPromise = new Promise((resolve, reject) => {
      uploadPngStream.on('finish', (uploadedFile) => {
        pngFileId = uploadedFile._id;
        resolve();
      });
      uploadPngStream.on('error', reject);
      uploadPngStream.end(pngBuffer);
    });

    await uploadPngPromise;

    const map = new Map({
      name,
      description,
      pgmFileId,
      yamlFileId,
      pngFileId
    });

    await map.save();
    console.log('Map saved to database:', map);
    res.status(201).json({ message: 'Map uploaded successfully', map });
  } catch (error) {
    console.error('Error uploading map:', error);
    res.status(500).json({ message: 'Error uploading map', error: error.message });
  }
};

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

exports.selectMapForMonitoring = async (req, res) => {
  try {
    const userId = req.user._id; 
    const { mapId } = req.body;

    await User.findByIdAndUpdate(userId, { selectedMapId: mapId });

    res.status(200).json({ message: 'Map selected for monitoring successfully' });
  } catch (error) {
    res.status(500).json({ message: 'Error selecting map for monitoring', error: error.message });
  }
};
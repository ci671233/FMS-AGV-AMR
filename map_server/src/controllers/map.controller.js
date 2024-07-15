const mongoose = require('mongoose');
const Grid = require('gridfs-stream');
const Map = require('../models/map.model');
const conn = mongoose.connection;
Grid.mongo = mongoose.mongo;
let gfs;

conn.once('open', () => {
    gfs = Grid(conn.db);
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
        const { name, meta } = req.body;
        const file = req.file;

        if (!file) {
            return res.status(400).json({ message: 'No file uploaded' });
        }

        const writeStream = gfs.createWriteStream({
            filename: file.originalname,
            content_type: file.mimetype
        });

        writeStream.on('close', async (file) => {
            const map = new Map({
                name,
                meta: JSON.parse(meta),
                fileId: file._id
            });

            await map.save();
            res.status(201).json({ message: 'Map uploaded successfully', map });
        });

        writeStream.write(file.buffer);
        writeStream.end();
    } catch (error) {
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

        gfs.files.findOne({ _id: mongoose.Types.ObjectId(map.fileId) }, (err, file) => {
            if (err || !file) {
                return res.status(404).json({ message: 'Map file not found' });
            }

            const readStream = gfs.createReadStream({ _id: file._id });
            let data = '';

            readStream.on('data', (chunk) => {
                data += chunk;
            });

            readStream.on('end', () => {
                const base64Image = Buffer.from(data).toString('base64');
                res.json({ mapId: selectedMapId, mapImage: base64Image, mapMeta: map.meta });
            });

            readStream.on('error', (err) => {
                res.status(500).json({ message: 'Error reading map file', error: err.message });
            });
        });
    } catch (error) {
        res.status(500).json({ message: 'Error fetching selected map', error: error.message });
    }
};


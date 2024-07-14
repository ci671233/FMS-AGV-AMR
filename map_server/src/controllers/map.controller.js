const fs = require('fs');
const path = require('path');
const Map = require('../models/map.model');

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

exports.getSelectedMap = async (req, res) => {
    try {
        const selectedMapId = req.user.selectedMapId; // 사용자가 선택한 맵 ID
        const map = await Map.findById(selectedMapId);
        if (!map) {
            return res.status(404).json({ message: 'Selected map not found' });
        }
        const mapPath = path.join(__dirname, '../../maps', `${map.fileName}.pgm`);
        fs.readFile(mapPath, (err, data) => {
            if (err) {
                return res.status(500).json({ message: 'Error reading map image', error: err.message });
            }
            const base64Image = Buffer.from(data).toString('base64');
            res.json({ mapId: selectedMapId, mapImage: base64Image, mapMeta: map.meta });
        });
    } catch (error) {
        res.status(500).json({ message: 'Error fetching selected map', error: error.message });
    }
};

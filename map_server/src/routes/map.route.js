const express = require('express');
const router = express.Router();
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/maps', auth, mapController.getMaps);
router.post('/upload', auth, mapController.uploadMap);
router.put('/update/:id', auth, mapController.updateMap);
router.get('/monitored', auth, mapController.getMonitoredMap);
router.get('/file/:id', auth, mapController.getMonitoredMapFile);

module.exports = router;


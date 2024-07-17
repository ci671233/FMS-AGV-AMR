const express = require('express');
const router = express.Router();
const multer = require('multer');
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth');

const storage = multer.memoryStorage();
const upload = multer({ storage }).fields([{ name: 'pgm', maxCount: 1 }, { name: 'yaml', maxCount: 1 }]);

router.post('/upload', upload, mapController.uploadMap);
router.get('/selected', auth.authenticate, mapController.getSelectedMap);
router.get('/', mapController.getMaps);
router.get('/:id', mapController.getMapById);
router.get('/png/:id', mapController.getPngMapById);
router.post('/select', auth.authenticate, mapController.selectMapForMonitoring);


module.exports = router;


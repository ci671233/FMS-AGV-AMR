const express = require('express');
const router = express.Router();
const multer = require('multer');
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth').authenticate;

const storage = multer.memoryStorage();
// const upload = multer({ storage }).fields([{ name: 'pgm', maxCount: 1 }, { name: 'yaml', maxCount: 1 }]);


router.get('/maps', auth, mapController.getMaps);
router.post('/save_map', auth, mapController.saveMap);


// router.post('/upload', upload, mapController.uploadMap);
// router.get('/', auth, mapController.getMaps);
// router.get('/:id', auth, mapController.getMapById);
// router.get('/png/:id', auth, mapController.getPngMapById);
// router.post('/select', auth, mapController.selectMapForMonitoring);
// router.get('/selected/:id', auth, mapController.getSelectedMapImage);

module.exports = router;


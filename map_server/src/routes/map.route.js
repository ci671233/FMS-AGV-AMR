const express = require('express');
const router = express.Router();
const multer = require('multer');
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth');

const storage = multer.memoryStorage();
const upload = multer({ storage });

router.post('/upload', upload.single('file'), mapController.uploadMap);
router.get('/selected', mapController.getSelectedMap);
router.get('/', mapController.getMaps);
router.get('/:id', mapController.getMapById);
router.get('/download/:id', mapController.downloadMap);


module.exports = router;


const express = require('express');
const router = express.Router();
const mapController = require('../controllers/map.controller');
const auth = require('../middlewares/auth');

router.get('/', mapController.getMaps);
router.get('/:id', mapController.getMapById);
router.get('/selected', mapController.getSelectedMap);

module.exports = router;

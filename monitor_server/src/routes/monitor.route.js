const express = require('express');
const router = express.Router();
const monitorController = require('../controllers/monitor.controller');

router.get('/positions', monitorController.getPositions);

module.exports = router;

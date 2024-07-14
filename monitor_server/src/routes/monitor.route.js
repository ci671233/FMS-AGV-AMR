const express = require('express');
const router = express.Router();
const monitorController = require('../controllers/monitor.controller');
const auth = require('../middlewares/auth');

router.get('/data', monitorController.getMonitoringData);

module.exports = router;


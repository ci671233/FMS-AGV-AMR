const express = require('express');
const router = express.Router();
const monitorController = require('../controllers/monitor.controller');
const authMiddleware = require('../middlewares/auth');

router.get('/data', authMiddleware.authenticate, monitorController.getMonitoringData);

module.exports = router;


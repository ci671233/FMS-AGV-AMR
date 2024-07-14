const express = require('express');
const router = express.Router();
const logController = require('../controllers/log.controller');
const authMiddleware = require('../middlewares/auth');

router.get('/latest', authMiddleware.authenticate, logController.getLatestLogs);

module.exports = router;

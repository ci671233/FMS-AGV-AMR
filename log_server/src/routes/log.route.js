const express = require('express');
const router = express.Router();
const logController = require('../controllers/log.controller');
const auth = require('../middlewares/auth');

router.get('/latest', logController.getLatestLogs);

module.exports = router;

const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const auth = require('../middlewares/auth');

router.post('/move/:robotID', auth.authenticate, robotController.moveRobot);
router.get('/status/:robotID', auth.authenticate, robotController.getRobotStatus);

module.exports = router;
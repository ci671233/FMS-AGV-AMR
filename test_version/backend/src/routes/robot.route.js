const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
//const auth = require('../middlewares/auth');

router.post('/create', robotController.createRobot);
router.post('/move/:robotID', robotController.moveRobot);
router.get('/status/:robotID', robotController.getRobotStatus);

// router.post('/create', auth.authenticate, robotController.createRobot);
// router.post('/move/:robotID', auth.authenticate, robotController.moveRobot);
// router.get('/status/:robotID', auth.authenticate, robotController.getRobotStatus);
module.exports = router;
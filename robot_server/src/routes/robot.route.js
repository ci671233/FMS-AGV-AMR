const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/robots', auth, robotController.getRobots);
router.post('/register_robot', auth, robotController.registerRobot);
// router.post('/send_command', auth, robotController.sendCommand);

module.exports = router;


const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/robots', auth, robotController.getRobots);
router.post('/register_robot', auth, robotController.registerRobot);
router.post('/send_command', auth, robotController.sendCommand);

// router.get('/positions/:id', auth, robotController.getRobotPositions);
// router.get('/:id', auth, robotController.getRobotById);

module.exports = router;

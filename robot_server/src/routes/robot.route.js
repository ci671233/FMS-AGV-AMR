const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const auth = require('../middlewares/auth').authenticate;

router.get('/robots', auth, robotController.getRobots);
router.post('/register', auth, robotController.registerRobot);
router.put('/update/:id', auth, robotController.updateRobot);
router.post('/send_map', auth, robotController.sendMapToRobots);

module.exports = router;



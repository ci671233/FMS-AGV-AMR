const express = require('express');
const router = express.Router();
const robotController = require('../controllers/robot.controller');
const auth = require('../middlewares/auth');

router.get('/positions', robotController.getRobotPositions);
router.get('/:id', robotController.getRobotById);

module.exports = router;

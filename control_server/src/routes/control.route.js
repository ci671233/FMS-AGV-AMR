const express = require('express');
const router = express.Router();
const controlController = require('../controllers/control.controller');
const authMiddleware = require('../middlewares/auth');

router.post('/control/:id', authMiddleware.authenticate, controlController.controlRobot);

module.exports = router;


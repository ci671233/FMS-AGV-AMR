const express = require('express');
const router = express.Router();
const controlController = require('../controllers/control.controller');
const auth = require('../middlewares/auth');

router.post('/control/:id', controlController.controlRobot);

module.exports = router;


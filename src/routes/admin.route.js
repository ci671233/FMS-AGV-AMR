const express = require('express');
const router = express.Router();
const adminController = require('../controllers/admin.controller');
const authMiddleware = require('../middlewares/auth');

router.put('/approve/:accountId', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.approveAccount);
router.put('/assign-role/:accountId', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.assignRole);

module.exports = router;
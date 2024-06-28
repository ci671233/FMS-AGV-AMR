const express = require('express');
const router = express.Router();
const factoryController = require('../controllers/factory.controller');
const authMiddleware = require('../middlewares/auth');

router.post('/create', authMiddleware.authenticate, authMiddleware.isSystemAdmin, factoryController.createFactory);
router.put('/update/:factoryId', authMiddleware.authenticate, authMiddleware.isFactoryAdmin, factoryController.updateFactory);
router.put('/add-user/:factoryId/:userId', authMiddleware.authenticate, authMiddleware.isFactoryAdmin, factoryController.addUserToFactory);
router.put('/add-admin/:factoryId/:adminId', authMiddleware.authenticate, authMiddleware.isSystemAdmin, factoryController.addAdminToFactory);

module.exports = router;

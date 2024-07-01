const express = require('express'); // express 패키지 로드
const router = express.Router();
const adminController = require('../controllers/admin.controller'); // /controllers/admin.controller 로드
const authMiddleware = require('../middlewares/auth'); // /middlewares/auth 로드

// 가입 승인
router.get('/pending/account', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.getPendingAccounts);
router.post('/approve/account/:accountID', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.approveAccount);

// 권한 승인
router.get('/pending/role', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.getPendingRoleRequests);
router.post('/approve/role/:accountID', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.approveRoleRequest);

// 공장 승인
router.get('/pending/factory', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.getPendingFactories);
router.post('/approve/factory/:factoryID', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.approveFactory);

module.exports = router;
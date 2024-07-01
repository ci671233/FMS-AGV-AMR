const express = require('express'); // express 패키지 로드
const router = express.Router();
const adminController = require('../controllers/admin.controller'); // /controllers/admin.controller 로드
const authMiddleware = require('../middlewares/auth'); // /middlewares/auth 로드

// 계정 상태 업데이트 함수
router.put('/approve/:accountID', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.approveAccount);
// 역할 할당 업데이트 함수
router.put('/assign_role/:accountID', authMiddleware.authenticate, authMiddleware.isSystemAdmin, adminController.assignRole);

module.exports = router;
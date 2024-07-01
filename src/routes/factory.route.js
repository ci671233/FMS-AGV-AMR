const express = require('express'); // express 패키지 로드
const router = express.Router();
const factoryController = require('../controllers/factory.controller'); // /controllers/factory.controller 로드
const authMiddleware = require('../middlewares/auth'); // /middlewares/auth 로드

// 공장 생성 (시스템 관리자만 접근 가능)
router.post('/create', authMiddleware.authenticate, authMiddleware.isSystemAdmin, factoryController.createFactory);
// 공장 정보 업데이트 (공장 관리자만 접근 가능)
router.put('/update/:factoryID', authMiddleware.authenticate, authMiddleware.isFactoryAdmin, factoryController.updateFactory);
// 공장에 사용자 추가 (공장 관리자만 접근 가능)
router.put('/add_user/:factoryID/:userID', authMiddleware.authenticate, authMiddleware.isFactoryAdmin, factoryController.addUserToFactory);
// 공장에 공장 관리자 권한 부여 (공장 관리자만 접근 가능)
router.put('/add_admin/:factoryID/:adminID', authMiddleware.authenticate, authMiddleware.isFactoryAdmin, factoryController.addAdminToFactory);

module.exports = router;

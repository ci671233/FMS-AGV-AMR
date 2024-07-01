const express = require('express'); // express 패키지 로드
const router = express.Router(); 
const accountController = require('../controllers/account.controller'); // /controllers/account.controller 로드
const validationMiddleware = require('../middlewares/validation'); // /middlewares/validation 로드

// 회원가입 함수
router.post('/register', validationMiddleware.validateRegister, accountController.register); 
// login함수
router.post('/login', accountController.login); 
// 새로운 엑세스 토큰 발급
router.post('/newaccesstoken', accountController.newAccessToken); 

module.exports = router;
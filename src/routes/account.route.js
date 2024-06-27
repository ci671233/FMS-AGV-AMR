const express = require('express'); // express 패키지 로드
const router = express.Router(); 
const accountController = require('../controllers/account.controller'); // /controllers/account.controller 로드
const validation = require('../middlewares/validation'); // /middlewares/validation 로드

router.post('/register', validation.validateRegister, accountController.register); // 회원가입 함수
router.post('/login', accountController.login); // login함수
router.post('/newaccesstoken', accountController.newAccessToken); // 새로운 엑세스 토큰 발급

module.exports = router;
const express = require('express'); // express 패키지 로드
const router = express.Router(); 
const accountController = require('../controllers/account.controller'); // /controllers/account.controller 로드
const auth = require('../middlewares/auth');

// 회원가입 함수
router.post('/register', accountController.register);  
// login함수
router.post('/login', accountController.login); 
// logout함수
router.post('/logout', accountController.logout); 
// 사용자 정보 함수
router.get('/userinfo', auth.authenticate, accountController.getUserInfo);
// 사용자 정보를 가져오는 라우트
router.get('/user/:id', auth.authenticate, accountController.getUserById);

module.exports = router;
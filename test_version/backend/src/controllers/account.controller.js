const Account = require('../models/account.model'); // /models/account.model 로드
// const jwt = require('../utils/jwt'); // /utils/jwt 로드
const passwordUtils = require('../utils/password');
const bcrypt = require('bcryptjs'); // bcryptjs 패키지 로드


// 회원가입 함수
exports.register = async (req, res) => {
    try {
        console.log('Received register request:', req.body); // 로그 추가
        const { email, password, name, phone } = req.body; // 양식: 이메일, 패스워드, 이름, 휴대전화

        // 사용자 존재 여부 확인
        const existingAccount = await Account.findOne({ email }); // 동일한 사용자 확인
        if (existingAccount) {
            return res.status(400).json({ message: 'Email already in use' }); // 이미 존재
        }

        // 비밀번호 해시
        const hashedPassword = await bcrypt.hash(password, 10);

        // 계정 생성
        const account = new Account({
            email,
            password: hashedPassword, // 비밀번호 해시
            name,
            phone,
            status: 'pending' // 기본 상태를 'pending'으로 설정
        });

        await account.save();
        res.status(201).json({ message: 'Account created. Waiting for approval.' });
    } catch (error) {
        console.error('Error during registration:', error); // 에러 로그 추가
        res.status(500).json({ message: 'Server error' });
    }
};

// 로그인 함수
exports.login = async (req, res) => {
    try {
        const { email, password } = req.body; // 양식: 이메일, 패스워드
        const account = await Account.findOne({ email }); // 이메일 탐색
        if (!account || !await bcrypt.compare(password, account.password)) {
            return res.status(401).send('Invalid email or password'); // 비밀번호 불일치
        }

        // const accessToken = jwt.generateToken({ id: account._id, role: account.role }, '1d');   // 엑세스 토큰은 1일 동안 유효  
        // const refreshToken = jwt.generateToken({ id: account._id, role: account.role }, '30d');  // 리프레시 토큰은 30일 동안 유효

        // res.cookie('refreshToken', refreshToken, { httpOnly: true, secure: true, sameSite: 'Strict' }); // 리프레시 토큰을 HTTP-only 쿠키에 저장
        res.status(200).send({ message: 'login.' }); // 엑세스 토큰 응답
    } catch (error) {
        res.status(400).send(error.message); // 에러 메시지
    }
};

// // 엑세스 토큰 재발급 함수
// exports.newAccessToken = async (req, res) => {
//     try {
//         const refreshToken = req.cookies.refreshToken; // 쿠키에서 리프레시 토큰 로드
//         const decoded = jwt.verifyToken(refreshToken); // 토큰 디코딩

//         if (!decoded) {
//             return res.status(401).send('Invalid refresh token'); // 토큰이 유효하지 않음
//         }

//         const accessToken = jwt.generateToken({ id: decoded.id, role: decoded.role }, '1d'); 
//         res.status(200).send({ accessToken });
//     } catch (error) {
//         res.status(400).send(error.message);
//     }
// };
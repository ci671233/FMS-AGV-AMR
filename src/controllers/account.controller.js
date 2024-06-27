const Account = require('../models/account.model'); // /models/account.model 로드
const jwt = require('../utils/jwt'); // /utils/jwt 로드
const passwordUtils = require('../utils/password');
const bcrypt = require('bcryptjs'); // bcryptjs 패키지 로드


// 회원가입 함수
exports.register = async (req, res) => {
    try {
        const { email, password, name, phone, role = 'user' } = req.body;
        const hashedPassword = await passwordUtils.hashPassword(password); // 비밀번호 해시
        const newAccount = new Account({ email, password: hashedPassword, name, phone, role });
        await newAccount.save();
        res.status(201).send('Account created successfully');
    } catch (error) {
        res.status(400).send(error.message);
    }
};

// 로그인 함수
exports.login = async (req, res) => {
    try {
        // 요청 email, password
        const { email, password } = req.body;
        const account = await Account.findOne({ email });
        if (!account || !await bcrypt.compare(password, account.password)) {
            return res.status(401).send('Invalid email or password');
        }

        const accessToken = jwt.generateToken({ id: account._id, role: account.role }, '1h');   // 엑세스 토큰은 1시간 동안 유효  
        const refreshToken = jwt.generateToken({ id: account._id, role: account.role }, '7d');  // 리프레시 토큰은 7일 동안 유효

        res.cookie('refreshToken', refreshToken, { httpOnly: true, secure: true, sameSite: 'Strict' }); // 리프레시 토큰을 HTTP-only 쿠키에 저장
        res.status(200).send({ accessToken });
    } catch (error) {
        res.status(400).send(error.message);
    }
};

// 엑세스 토큰 재발급 함수
exports.newAccessToken = async (req, res) => {
    try {
        const refreshToken = req.cookies.refreshToken;
        const decoded = jwt.verifyToken(refreshToken);

        if (!decoded) {
            return res.status(401).send('Invalid refresh token');
        }

        const accessToken = jwt.generateToken({ id: decoded.id, role: decoded.role }, '1h');
        res.status(200).send({ accessToken });
    } catch (error) {
        res.status(400).send(error.message);
    }
};
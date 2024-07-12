const { body, validationResult } = require('express-validator');
const { isEmailValid, isPhoneNumberValid } = require('../utils/validate');
const { formatPhoneNumber } = require('../utils/format');

// 회원가입 요청 데이터를 검증하는 미들웨어
exports.validateRegister = [
    body('email').custom(value => isEmailValid(value)).withMessage('Invalid email'), // 이메일 유효성 검사 
    body('password').isLength({ min: 6 }).withMessage('Password must be at least 6 characters long'), // 패스워드 최소 길이
    body('name').notEmpty().withMessage('Name is required'), // name 요청
    body('phone').custom(value => isPhoneNumberValid(value)).withMessage('Invalid phone number'), // 전화번호 유효성 검사
    (req, res, next) => {
        const errors = validationResult(req);
        if (!errors.isEmpty()) {
            return res.status(400).json({ errors: errors.array() });
        }
        req.body.phone = formatPhoneNumber(req.body.phone); // 전화번호 포맷팅
        next();
    }
];

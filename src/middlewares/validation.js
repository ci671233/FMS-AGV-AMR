const { body, validationResult } = require('express-validator');
const { isEmailValid, isPhoneNumberValid } = require('../utils/validate');
const { formatPhoneNumber } = require('../utils/format');

// 회원가입 요청 데이터를 검증하는 미들웨어
exports.validateRegister = [
    body('email').custom(value => isEmailValid(value)).withMessage('Invalid email'),
    body('password').isLength({ min: 6 }).withMessage('Password must be at least 6 characters long'),
    body('name').notEmpty().withMessage('Name is required'),
    body('phone').custom(value => isPhoneNumberValid(value)).withMessage('Invalid phone number'),
    (req, res, next) => {
        const errors = validationResult(req);
        if (!errors.isEmpty()) {
            return res.status(400).json({ errors: errors.array() });
        }
        req.body.phone = formatPhoneNumber(req.body.phone); // 전화번호 포맷팅
        next();
    }
];

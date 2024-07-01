const jwt = require('../utils/jwt');
const Account = require('../models/account.model');

// 사용자가 인증되었는지 확인
// exports.authenticate = async (req, res, next) => {
//     try {
//         const authHeader = req.header('Cookie');
//         if (!authHeader) {
//             return res.status(401).send({ error: 'Authorization header missing' });
//         }

//         const token = authHeader.replace('refreshToken=', ''); // 요청 헤더에서 토큰 추출 및 접두사 제거
//         const decoded = jwt.verifyToken(token); // 토큰 확인

//         if (!decoded) {
//             // 토큰이 유효하지 않은 경우 401 Unauthorized 응답
//             return res.status(401).send({ error: 'Invalid token' });
//         }

//         // 토큰에서 추출한 ID로 사용자를 데이터베이스에서 검색
//         const account = await Account.findById(decoded.id);
//         if (!account) {
//             // 사용자를 찾을 수 없는 경우 401 Unauthorized 응답
//             return res.status(401).send({ error: 'Account not found' });
//         }

//         // 인증된 사용자를 요청 객체에 추가
//         req.user = account;
//         next(); // 다음 미들웨어 또는 라우트 핸들러로 제어 전달
//     } catch (error) {
//         res.status(500).send({ error: 'Server error' });
//     }
// };

// 인증 미들웨어: 사용자가 인증되었는지 확인
exports.authenticate = async (req, res, next) => {
    try {
        const token = req.header('Authorization').replace('Bearer ', '');
        const decoded = jwt.verify(token, process.env.JWT_SECRET);

        const user = await Account.findById(decoded.id);
        if (!user) {
            return res.status(401).send({ error: 'Please authenticate.' });
        }

        req.user = user;
        next();
    } catch (error) {
        res.status(401).send({ error: 'Please authenticate.' });
    }
};

// 시스템 관리자 권한 확인
exports.isSystemAdmin = (req, res, next) => {
    if (req.user.role !== 'system_admin') {
        // 권한이 없을 경우 거부 응답
        return res.status(403).send({ error: 'Access denied.' });
    }
    next();
};

// 공장에 접근 권한 확인
exports.isFactoryAdmin = (req, res, next) => {
    const { factoryID } = req.params;
    if (req.user.role === 'system_admin' || (req.user.role === 'factory_admin' && req.user.factories.includes(factoryID))) {
        return next(); // 권한이 있는 경우 다음 미들웨어 또는 라우트 핸들러로 제어 전달
    }
    // 권한이 없는 경우 거부 응답
    return res.status(403).send({ error: 'Access denied.' });
};
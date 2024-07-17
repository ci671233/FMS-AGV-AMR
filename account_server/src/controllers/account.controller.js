const Account = require('../models/account.model'); // /models/account.model 로드
const jwtUtils = require('../utils/jwt'); // /utils/jwt 로드
const hashUtils = require('../utils/hash');

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
        const hashedPassword = await hashUtils.hashPassword(password);

        // 계정 생성
        const account = new Account({
            email,
            password: hashedPassword, // 비밀번호 해시
            name,
            phone
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
        const { email, password } = req.body;

        const account = await Account.findOne({ email });
        if (!account) {
            return res.status(400).json({ message: 'Invalid email or password' });
        }

        const isPasswordValid = await hashUtils.comparePassword(password, account.password);
        if (!isPasswordValid) {
            return res.status(400).json({ message: 'Invalid email or password' });
        }

        const token = jwtUtils.generateToken({ email: account.email, id: account._id, name: account.name }, '1h');
        res.json({ token, name: account.name });
    } catch (error) {
        console.error('Error during login:', error);
        res.status(500).json({ message: 'Server error', error: error.message });
    }
};

// 로그아웃 함수
exports.logout = async (req, res) => {
    try {
        // 클라이언트 측에서 토큰을 삭제하면 로그아웃이 됩니다.
        res.json({ message: 'Logged out' });
    } catch (error) {
        console.error('Error during logout:', error);
        res.status(500).json({ message: 'Server error' });
    }
};

// 사용자 정보 조회 함수
exports.getUserInfo = async (req, res) => {
    try {
        const token = req.headers.authorization?.split(' ')[1];
        if (!token) {
            return res.status(401).json({ message: 'Authentication token is required' });
        }

        const payload = jwtUtils.verifyToken(token);
        if (!payload) {
            return res.status(401).json({ message: 'Invalid token' });
        }

        res.json({ name: payload.name });
    } catch (error) {
        console.error('Error fetching user info:', error);
        res.status(500).json({ message: 'Server error' });
    }
};

exports.getUserById = async (req, res) => {
    try {
      const userId = req.params.id;
      const user = await User.findById(userId);
      if (!user) {
        return res.status(404).json({ message: 'User not found' });
      }
      res.json(user);
    } catch (error) {
      res.status(500).json({ message: 'Error fetching user', error: error.message });
    }
  };
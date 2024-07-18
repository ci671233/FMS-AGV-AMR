const jwt = require('jsonwebtoken');
const axios = require('axios');

exports.authenticate = async (req, res, next) => {
  try {
    const authHeader = req.headers['authorization'];
    if (!authHeader) {
      console.log('No Authorization header found.');
      return res.status(401).json({ message: 'Access denied. No token provided.' });
    }

    const token = authHeader.split(' ')[1];
    console.log('Token received:', token);

    const decoded = jwt.verify(token, process.env.JWT_SECRET_KEY);
    req.user = decoded;
    console.log('Decoded token:', decoded);

    // account_server에서 사용자 정보 확인
    const response = await axios.get(`http://localhost:5555/account/user/${req.user.id}`, {
      headers: { Authorization: `Bearer ${token}` }
    });

    if (response.status !== 200) {
      console.log('Invalid token.');
      return res.status(401).json({ message: 'Invalid token.' });
    }

    console.log('User info from account server:', response.data);
    req.user = response.data;
    next();
  } catch (error) {
    console.error('Error in authenticate middleware:', error.message);
    res.status(401).json({ message: 'Invalid token.' });
  }
};



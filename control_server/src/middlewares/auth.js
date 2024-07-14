const { verifyToken } = require('../utils/jwt');

exports.authenticate = async (req, res, next) => {
    try {
        const token = req.headers.authorization?.split(' ')[1];

        if (!token) {
            return res.status(401).json({ message: 'Authentication token is required' });
        }

        const payload = verifyToken(token);
        if (!payload) {
            return res.status(401).json({ message: 'Invalid token' });
        }

        req.user = payload;
        next();

    } catch (error) {
        res.status(401).send({ error: 'Please authenticate.' });
    }
};

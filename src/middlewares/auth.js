const jwt = require('../utils/jwt');
const Account = require('../models/account.model');

exports.authenticate = async (req, res, next) => {
    const token = req.header('Authorization').replace('Bearer ', '');
    const decoded = jwt.verifyToken(token);

    if (!decoded) {
        return res.status(401).send({ error: 'Please authenticate.' });
    }

    const account = await Account.findById(decoded.id);
    if (!account) {
        return res.status(401).send({ error: 'Please authenticate.' });
    }

    req.user = account;
    next();
};

exports.isSystemAdmin = (req, res, next) => {
    if (req.user.role !== 'system_admin') {
        return res.status(403).send({ error: 'Access denied.' });
    }
    next();
};

exports.isFactoryAdmin = (req, res, next) => {
    const { factoryId } = req.params;
    if (req.user.role === 'system_admin' || (req.user.role === 'factory_admin' && req.user.factories.includes(factoryId))) {
        return next();
    }
    return res.status(403).send({ error: 'Access denied.' });
};
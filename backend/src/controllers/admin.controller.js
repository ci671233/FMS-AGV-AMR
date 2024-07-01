const Account = require('../models/account.model'); // /models/account.model 로드
const Factory = require('../models/factory.model'); // /models/factory.model 로드

// 대기 중인 계정 승인 목록 조회
exports.getPendingAccounts = async (req, res) => {
    try {
        const accounts = await Account.find({ status: 'pending' });
        res.status(200).json(accounts);
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 계정 승인
exports.approveAccount = async (req, res) => {
    try {
        const { accountID } = req.params;
        const account = await Account.findByIdAndUpdate(accountID, { status: 'approved' }, { new: true });
        if (!account) {
            return res.status(404).json({ message: 'Account not found' });
        }
        res.status(200).json({ message: 'Account approved.', account });
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 대기 중인 역할 변경 요청 목록 조회
exports.getPendingRoleRequests = async (req, res) => {
    try {
        const accounts = await Account.find({ roleRequest: { $ne: null } });
        res.status(200).json(accounts);
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 역할 변경 승인
exports.approveRoleRequest = async (req, res) => {
    try {
        const { accountID } = req.params;
        const account = await Account.findById(accountID);
        if (!account) {
            return res.status(404).json({ message: 'Account not found' });
        }
        account.role = account.roleRequest;
        account.roleRequest = null;
        await account.save();
        res.status(200).json({ message: 'Role request approved.', account });
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 대기 중인 공장 목록 조회
exports.getPendingFactories = async (req, res) => {
    try {
        const factories = await Factory.find({ status: 'pending' });
        res.status(200).json(factories);
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 공장 승인
exports.approveFactory = async (req, res) => {
    try {
        const { factoryID } = req.params;
        const factory = await Factory.findByIdAndUpdate(factoryID, { status: 'approved' }, { new: true });
        if (!factory) {
            return res.status(404).json({ message: 'Factory not found' });
        }
        res.status(200).json({ message: 'Factory approved.', factory });
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};
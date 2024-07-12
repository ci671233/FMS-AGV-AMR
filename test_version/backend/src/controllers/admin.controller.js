const Account = require('../models/account.model'); // /models/account.model 로드
const Factory = require('../models/factory.model'); // /models/factory.model 로드

// 대기 중인 계정 승인 목록 조회
exports.getPendingAccounts = async (req, res) => {
    try {
        const accounts = await Account.find({ status: 'pending' }); // 대기중인 계정 로드
        res.status(200).json(accounts);
    } catch (error) {
        res.status(500).json({ message: 'Server error' }); // 에러 출력
    }
};

// 계정 승인
exports.approveAccount = async (req, res) => {
    try {
        const { accountID } = req.params;
        const account = await Account.findByIdAndUpdate(accountID, { status: 'approved' }, { new: true }); // 활성화 상태로 변경
        if (!account) {
            return res.status(404).json({ message: 'Account not found' }); // 계정 찾을 수 없음
        }
        res.status(200).json({ message: 'Account approved.', account }); // 계정 승인 응답
    } catch (error) {
        res.status(500).json({ message: 'Server error' }); // 에러 출력
    }
};

// 대기 중인 역할 변경 요청 목록 조회
exports.getPendingRoleRequests = async (req, res) => {
    try {
        const accounts = await Account.find({ roleRequest: { $ne: null } }); // 요청 있는 계정 확인 null이 아닌 계정
        res.status(200).json(accounts);
    } catch (error) {
        res.status(500).json({ message: 'Server error' });
    }
};

// 역할 변경 승인
exports.approveRoleRequest = async (req, res) => {
    try {
        const { accountID } = req.params; // 요청 값 accountID
        const account = await Account.findById(accountID); // 해당 계정 검색
        if (!account) {
            return res.status(404).json({ message: 'Account not found' }); // 계정을 찾을 수 없음
        }
        account.role = account.roleRequest; // 요청한 역할 부여
        account.roleRequest = null; // 요청상태 초기화
        await account.save();
        res.status(200).json({ message: 'Role request approved.', account }); // 성공 응답
    } catch (error) {
        res.status(500).json({ message: 'Server error' }); // 오류 출력
    }
};

// 대기 중인 공장 목록 조회
exports.getPendingFactories = async (req, res) => {
    try {
        const factories = await Factory.find({ status: 'pending' }); // 대기중인 공장 추가 목록 조회
        res.status(200).json(factories);
    } catch (error) {
        res.status(500).json({ message: 'Server error' }); // 오류 출력
    }
};

// 공장 승인
exports.approveFactory = async (req, res) => {
    try {
        const { factoryID } = req.params;
        const factory = await Factory.findByIdAndUpdate(factoryID, { status: 'approved' }, { new: true }); // 승인 상태로 변경
        if (!factory) {
            return res.status(404).json({ message: 'Factory not found' }); // 공장을 찾을 수 없음
        }
        res.status(200).json({ message: 'Factory approved.', factory }); // 승인 출력
    } catch (error) {
        res.status(500).json({ message: 'Server error' }); // 오류 출력
    }
};
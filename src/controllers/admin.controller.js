const Account = require('../models/account.model'); // /models/account.model 로드

// 계정 승인 관련 함수
exports.approveAccount = async (req, res) => {
    try {
        const { accountId } = req.params;   // 요청 쿼리 accountId
        const account = await Account.findById(accountId); // AccountDB에서 해당 accountId 조회
        if (!account) {
            return res.status(404).send('Account not found');   // 조회 정보 없음
        }
        account.status = 'active';  // 계정 상태 활성화로 변경
        await account.save(); // 저장
        res.status(200).send('Account approved successfully'); // 승인 성공 응답
    } catch (error) {
        res.status(400).send(error.message); // 에러 메시지 
    }
};

// 계정 역할 관련 함수
exports.assignRole = async (req, res) => {
    try {
        const { accountId } = req.params; // 요청 쿼리 accountId
        const { role } = req.body;  // 요청 바디 role
        const account = await Account.findById(accountId);  // AccountDB에서 해당 accountId 조회
        if (!account) {
            return res.status(404).send('Account not found');   // 조회 정보 없음
        }
        account.role = role;    // 계정 역할 변경
        await account.save();   // 저장
        res.status(200).send('Role assigned successfully'); // 변경 성공 응답
    } catch (error) {
        res.status(400).send(error.message);    // 에러 메시지
    }
};
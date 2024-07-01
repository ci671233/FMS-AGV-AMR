const Factory = require('../models/factory.model'); // /models/factory.model 로드
const Account = require('../models/account.model'); // /models/account.model 로드

// 공장 등록 함수
exports.createFactory = async (req, res) => {
    try {
        const { name, location, size } = req.body;
        const createdBy = req.user._id; // 요청한 사용자 ID

        // 사용자 이름 추출
        const user = await Account.findById(createdBy);
        if (!user) {
            return res.status(404).json({ message: 'User not found' });
        }
        const createdByName = user.name;

        const newFactory = new Factory({
            name,
            location,
            size,
            description,
            createdBy,
            createdByName,
            status: 'pending' // 기본 상태를 'pending'으로 설정
        });

        await newFactory.save();
        res.status(201).json({ message: 'Factory created. Waiting for approval.', factory: newFactory });
    } catch (error) {
        console.error('Error creating factory:', error);
        res.status(500).json({ message: 'Server error' });
    }
};

// 공장 정보 업데이트
exports.updateFactory = async (req, res) => {
    try {
        const { factoryID } = req.params; // 요청에서 factoryID 추출
        const updates = req.body; // 요청 본문에서 update 정보 추출

        const updateFields = {}; // 업데이트할 필드만 포함된 객체를 만들기
        Object.keys(updates).forEach((key) => {
            if (updates[key] !== undefined) {
                updateFields[key] = updates[key];
            }
        });

        const factory = await Factory.findByIdAndUpdate(factoryID, updateFields, { new: true });

        if (!factory) {
            // 공장을 찾을 수 없는 경우
            return res.status(404).send('Factory not found');
        }
        // 완료 응답
        res.status(200).send(factory);
    } catch (error) {
        // 에러 응답
        res.status(400).send(error.message);
    }
};

// 공장에 사용자 등록 함수
exports.addUserToFactory = async (req, res) => {
    try {
        const { factoryID, userID } = req.params; // 요청에서 factoryID와 userID 추출
        const factory = await Factory.findById(factoryID);// factoryID로 공장 검색
        const user = await Account.findById(userID); // userID로 사용자 검색

        if (!factory || !user) {
            // 공장이나 사용자를 찾을 수 없는 경우
            return res.status(404).send('Factory or User not found');
        }

        factory.users.push(user._id); // 공장 사용자 목록에 사용자 추가
        user.factories.push(factory._id); // 사용자의 공장 목록에 공장 추가

        await factory.save(); // 공장 정보 저장
        await user.save(); // 사용자 정보 저장

        res.status(200).send('User added to factory successfully'); // 성공 응답
    } catch (error) {
        res.status(400).send(error.message); // 오류 응답
    }
};

// 공장에 관리자 권한 부여 함수
exports.addAdminToFactory = async (req, res) => {
    try {
        const { factoryID, adminID } = req.params; // 요청에서 factoryID와 adminID 추출
        const factory = await Factory.findById(factoryID); // factoryID로 공장 검색
        const admin = await Account.findById(adminID); // adminID로 사용자 검색

        if (!factory || !admin) {
            // 공장이나 사용자를 찾을 수 없는 경우
            return res.status(404).send('Factory or Admin not found');
        }

        factory.admins.push(admin._id); // 공장 관리자 목록에 사용자 추가
        admin.factories.push(factory._id); // 사용자의 공장 목록에 공장 추가

        await factory.save(); // 공장 정보 저장
        await admin.save(); // 사용자 정보 저장

        res.status(200).send('Admin added to factory successfully'); // 성공 응답
    } catch (error) {
        res.status(400).send(error.message); // 오류 응답
    }
};
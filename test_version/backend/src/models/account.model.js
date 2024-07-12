const mongoose = require('mongoose');   // mongoose 패키지를 로드

// accountSchema 정의
const accountSchema = new mongoose.Schema({
    email: { type: String, required: true, unique: true }, // 이메일 (아이디로 사용)
    password: { type: String, required: true }, // 패스워드
    name: { type: String, required: true }, // 사용자명
    phone: { type: String, required: true }, // 휴대전화 번호
    role: { type: String, enum: ['user', 'factory_admin', 'system_admin'], default: 'user' }, // 시스템 관리자, 공장 관리자, 유저가(기본값은 유저)
    status: { type: String, enum: ['pending', 'active'], default: 'pending' }, // 계정 상태 대기, 활성화 (기본 값 대기)
    roleRequest: { type: String, enum: ['user', 'factory_admin', 'system_admin'], default: null }, // 역할 변경 요청
    createdAt: { type: Date, default: Date.now }, // 계정 생성 시간
    factories: [{ type: mongoose.Schema.Types.ObjectId, ref: 'Factory' }] // 자신이 속해있는 공장
});

module.exports = mongoose.model('Account', accountSchema);
const mongoose = require('mongoose');

const factorySchema = new mongoose.Schema({
    name: { type: String, required: true }, // 공장 이름
    location: { type: String, required: true }, // 공장 위치
    size: { type: String, required: true }, // 공장 크기
    description: { type: String }, // 공장 세부설명
    createdAt: { type: Date, default: Date.now }, // 등록일자
    admins: [{ type: mongoose.Schema.Types.ObjectId, ref: 'Account' }], // 공장 관리자 추가
    users: [{ type: mongoose.Schema.Types.ObjectId, ref: 'Account' }]  // 일반 사용자 추가
});

module.exports = mongoose.model('Factory', factorySchema);

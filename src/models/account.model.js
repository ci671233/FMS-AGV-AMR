const mongoose = require('mongoose');   // mongoose 패키지를 로드

// accountSchema 정의
const accountSchema = new mongoose.Schema({
    email: { type: String, required: true, unique: true },
    password: { type: String, required: true },
    name: { type: String, required: true },
    phone: { type: String, required: true },
    role: { type: String, enum: ['admin', 'user'], default: 'user' }, 
    createdAt: { type: Date, default: Date.now },
    factories: [{ type: mongoose.Schema.Types.ObjectId, ref: 'Factory' }]
});

module.exports = mongoose.model('Account', accountSchema);
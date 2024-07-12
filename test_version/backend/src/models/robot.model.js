const mongoose = require('mongoose');

const robotSchema = new mongoose.Schema({
    name: { type: String, required: true },
    model: { type: String, required: true },
    batteryLevel: { type: Number, required: true }, // 배터리 잔량 (%)
    location: {
        x: { type: Number, required: true },
        y: { type: Number, required: true },
    },
    status: { type: String, enum: ['idle', 'moving', 'charging', 'error'], default: 'idle' },
    createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Robot', robotSchema);

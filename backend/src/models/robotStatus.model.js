const mongoose = require('mongoose');

const robotStatusSchema = new mongoose.Schema({
    robotId: { type: mongoose.Schema.Types.ObjectId, ref: 'Robot', required: true },
    status: { type: String, required: true }, // 예: 'idle', 'moving', 'error'
    batteryLevel: { type: Number, required: true }, // 배터리 잔량 (%)
    location: {
        x: { type: Number, required: true },
        y: { type: Number, required: true },
    },
    lastUpdated: { type: Date, default: Date.now }
});

module.exports = mongoose.model('RobotStatus', robotStatusSchema);

const mongoose = require('mongoose');

const robotSchema = new mongoose.Schema({
    name: { type: String, required: true },
    model: { type: String, required: true },
    battery: { type: Number, required: true },
    status: { type: String, required: true },
    position: {
        x: { type: Number, required: true },
        y: { type: Number, required: true }
    },
    mapId: { type: mongoose.Schema.Types.ObjectId, required: true }
});

module.exports = mongoose.model('Robot', robotSchema);

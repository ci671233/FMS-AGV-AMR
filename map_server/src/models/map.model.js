const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
    name: { type: String, required: true },
    meta: { type: Object, required: true },
    fileId: { type: mongoose.Schema.Types.ObjectId, required: true },
    createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Map', mapSchema);



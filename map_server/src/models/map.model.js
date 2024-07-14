const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
    name: String,
    fileName: String,
    meta: Object,
    createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Map', mapSchema);

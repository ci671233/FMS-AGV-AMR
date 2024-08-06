const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
  name: { type: String, required: true },
  description: { type: String, required: true },
  FileId: { type: mongoose.Schema.Types.ObjectId, required: true },
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Map', mapSchema);







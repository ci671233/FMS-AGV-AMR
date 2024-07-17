const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
  name: { type: String, required: true },
  description: { type: String, required: true },
  pgmFileId: { type: mongoose.Schema.Types.ObjectId, required: true },
  yamlFileId: { type: mongoose.Schema.Types.ObjectId, required: true },
  pngFileId: { type: mongoose.Schema.Types.ObjectId, required: true }, // 추가된 필드
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Map', mapSchema);







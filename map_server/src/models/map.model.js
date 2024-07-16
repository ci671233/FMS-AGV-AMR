const mongoose = require('mongoose');

const mapSchema = new mongoose.Schema({
  name: { type: String, required: true },
  description: { type: String, required: true },
  fileId: { type: mongoose.Schema.Types.ObjectId, required: true },
  fileName: { type: String, required: true },  // 파일 이름 필드 추가
  createdAt: { type: Date, default: Date.now }
});

module.exports = mongoose.model('Map', mapSchema);






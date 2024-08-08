require('dotenv').config();
const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');
const bodyParser = require('body-parser');
const cookieParser = require('cookie-parser');

// Routes 모듈을 로드, 관련 api
const monitorRoutes = require('./routes/monitor.route');

const app = express();

const MONGODB_URI = process.env.MONGODB_URI;
const allowedOrigins = process.env.FRONT_URI.split(',');

console.log("Allowed Origins:", allowedOrigins);  // 추가된 로그

// CORS 설정
app.use(cors({
  origin: (origin, callback) => {
    if (!origin || allowedOrigins.indexOf(origin) !== -1) {
      callback(null, true);
    } else {
      callback(new Error('Not allowed by CORS'));
    }
  },
  credentials: true
}));

// 요청 로그 출력
app.use((req, res, next) => {
  console.log(`${req.method} ${req.url}`);
  next();
});

app.use(bodyParser.json());
app.use(cookieParser());

mongoose.connect(MONGODB_URI, { useNewUrlParser: true, useUnifiedTopology: true })
    .then(() => console.log('MongoDB connected'))
    .catch(err => console.log(err));

app.use('/monitor', monitorRoutes);

app.use((err, req, res, next) => {
  console.error('Unexpected error:', err.stack);
  res.status(500).send('Something broke!');
});

module.exports = app;

// dotenv 패키지를 로드하고 .env 파일의 환경 변수를 process.env 객체에 로드
require('dotenv').config();

const express = require('express');     // express 패키지를 로드, 애플리케이션 생성
const mongoose = require('mongoose');   // mongoose 패키지를 로드
const bodyParser = require('body-parser');  // body-parser 패키지를 로드, 요청 본문을 파싱
const cookieParser = require('cookie-parser'); // cookie-parser 패키지를 로드, 쿠키 파싱
// Routes 모듈을 로드, 관련 api
const accountRoutes = require('./routes/account.route');
const adminRoutes = require('./routes/admin.route');
const factoryRoutes = require('./routes/factory.route');



const app = express();
const MONGODB_URI = process.env.MONGODB_URI;

app.use(bodyParser.json()); // 요청 본문 json으로 파싱
app.use(cookieParser()); // 쿠키 파싱

mongoose.connect(MONGODB_URI)
    .then(() => console.log('MongoDB connected'))   // 연결이 성공하면 콘솔에 메시지를 출력합니다.
    .catch(err => console.log(err));    // 연결이 실패하면 콘솔에 오류 메시지를 출력합니다.

// '/api/...' 경로로 들어오는 요청은 ...Routes 모듈에서
app.use('/api/account', accountRoutes);
app.use('/api/admin', adminRoutes);
app.use('/api/factory', factoryRoutes);


// app 모듈 내보내기
module.exports = app;
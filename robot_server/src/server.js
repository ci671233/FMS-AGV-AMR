// dotenv 패키지를 로드하고 .env 파일의 환경 변수를 process.env 객체에 로드
require('dotenv').config();

// app 로드
const { app, server } = require('./app'); // app과 server를 함께 가져옵니다.
// .env 파일에서 환경 변수 불러오기
const PORT = process.env.PORT;

// 서버를 지정된 포트에서 실행합니다.
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});
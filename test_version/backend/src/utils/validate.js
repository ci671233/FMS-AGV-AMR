// 이메일 유효성을 검증하는 함수
exports.isEmailValid = (email) => {
    const emailRegex = /^[a-zA-Z0-9._-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,4}$/;
    return emailRegex.test(email);
};
  
// 전화번호 유효성을 검증하는 함수
exports.isPhoneNumberValid = (phoneNumber) => {
    const phoneRegex = /^(01[016789]-?\d{3,4}-?\d{4})$/;
    return phoneRegex.test(phoneNumber);
};
  

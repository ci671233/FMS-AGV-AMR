// 전화번호를 포맷팅하는 함수
exports.formatPhoneNumber = (phoneNumber) => {
    if (phoneNumber) {
      const cleaned = phoneNumber.replace(/[^0-9]/g, '');
      let formatted = '';
  
      if (cleaned.length === 10) {
        formatted = cleaned.replace(/(\d{3})(\d{3,4})(\d{4})/, '$1-$2-$3');
      } 
      else if (cleaned.length === 11) {
        formatted = cleaned.replace(/(\d{3})(\d{4})(\d{4})/, '$1-$2-$3');
      } 
      else if (cleaned.length === 9) {
        formatted = cleaned.replace(/(\d{2})(\d{3})(\d{4})/, '$1-$2-$3');
      } 
      else {
        formatted = cleaned;
      }
      return formatted;
    }
    return '-';
};
  
  
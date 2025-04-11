# FMS-AGV-AMR
- (주) 한국에너지기술단과 국립한밭대학교 현장실습 프로그램을 통해서 참가한 프로젝트입니다. 초기 프로토타입 설계 및 구현 후 인수인계 업무를 할당받았습니다.
- AGV(자율 유도 차량) 및 AMR(자율 이동 로봇)을 효율적으로 **관리, 모니터링, 최적화**하기 위한 Fleet Management System입니다.

---

## 프로젝트 개요

- **프로젝트 목적**  
  산업 자동화와 물류 혁신의 흐름 속에서, AGV와 AMR의 체계적인 운영 및 관리 필요성이 증가하고 있습니다. 본 프로젝트는 AGV/AMR 로봇을 통합적으로 제어 및 모니터링하는 FMS(Fleet Management System)를 구현하는 것을 목표로 합니다.

- **적용 분야**  
  물류 센터, 스마트 공장, 병원 등 AGV/AMR이 사용되는 다양한 산업 현장

---

## 🛠️ 기술 스택 및 환경

- **ROS 버전**: Noetic (개발), ROS2 Foxy (인수인계 기준)
- **로봇 플랫폼**: TurtleBot3
- **운영체제**: Ubuntu
- **기타**: Raspberry Pi, OpenCR

---

## 서비스 벤치마킹

- 사내 자료로 인해 본 문서에는 포함되지 않습니다.

---

### 애플리케이션 구성도  
<img width="648" alt="애플리케이션 구성도" src="https://github.com/user-attachments/assets/d50381cd-e435-402c-8db1-84b17ecf5c13" />

### 시스템 구성도  
<img width="648" alt="시스템 구성도" src="https://github.com/user-attachments/assets/e24d0bef-0734-45a3-9f2e-ef2e14636e8d" />

### 초기 인터페이스 설계  
<img width="648" alt="UI 설계" src="https://github.com/user-attachments/assets/e0dcfd2d-b099-435e-a48d-e3e32b47ec8d" />

### 페이지 목록  
<img width="700" alt="페이지 목록" src="https://github.com/user-attachments/assets/98ada219-ce2f-474c-bcc6-34a93d005884" />

### 요구사항 정의서  
<img width="779" alt="요구사항 정의서" src="https://github.com/user-attachments/assets/7eb7f84c-c643-4d2c-b4f4-3401ebd0c00c" />

---

## 초기 프로토타입 결과물
### 로그인 및 회원가입  
<img width="649" alt="로그인 회원가입" src="https://github.com/user-attachments/assets/ef0135cd-aed3-4007-96ad-5ecda49e6ecd" />

### 메인 화면  
<img width="919" alt="메인 페이지" src="https://github.com/user-attachments/assets/9247697b-9482-4c34-95c8-77ccd9231945" />

### 맵 생성  
> RVIZ 화면 실시간 캡처 + WebCam 화면 실시간 스트리밍  
<img width="1002" alt="맵 생성" src="https://github.com/user-attachments/assets/7b2e8965-7788-483c-9a0a-d7ced82fcf84" />

### 맵 등록  
> 파일을 청크 단위로 나누어 DB에 삽입, 이름과 메타데이터 추가  
<img width="967" alt="맵 등록" src="https://github.com/user-attachments/assets/1d71bcc7-8e51-44ec-a555-dd23f70b49cc" />

### 로봇 등록  
> 로봇의 실시간 IP 주소 입력  
<img width="1016" alt="로봇 등록" src="https://github.com/user-attachments/assets/df788640-67ac-4151-8fc6-b9ec68abb759" />

### 맵 수정  
> PUT 요청으로 맵 정보 업데이트  
<img width="1134" alt="맵 수정" src="https://github.com/user-attachments/assets/9c3dbfc8-0767-43e3-a328-f143e9e2aa8f" />

### 맵 전송 및 노드 실행  
<img width="1016" alt="맵 전송 및 노드 실행" src="https://github.com/user-attachments/assets/ff4bc7ca-7fdc-47cc-afb9-96a4ef4945f4" />

### 모니터링 페이지  
> 현재 맵 상의 로봇 위치 데이터를 수신까지 구현 완료  
> *웹 기반 시각화는 추후 추가 예정*  
<img width="1181" alt="모니터링 페이지" src="https://github.com/user-attachments/assets/c394e7fb-095f-484e-a890-6d1a003137bb" />

---
## 참고 자료

- 🔗 [TurtleBot3 e-Manual (Noetic)](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- 본 프로젝트는 **ROS Noetic 기반으로 개발**되었으며,  
  **ROS2 Foxy 기준으로 인수인계 자료가 함께 정리**되어 있습니다.

---

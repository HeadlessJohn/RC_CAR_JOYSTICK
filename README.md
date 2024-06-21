# RC Car Project
## Nucleo F411RE 보드를 이용한 RC Car
### 1. 기능
- 블루투스로 연결된 조이스틱으로부터 ADC 값 수신 [`RC_CAR_Controller`](https://github.com/HeadlessJohn/RC_CAR_CONTROLLER)
- 수신된 값을 계산하여 좌륜, 우륜 속도 분배
- LCD에 상태 표시
### 2. 사용한 부품
- 
- DC모터 4개
- `L298N` 모터 드라이버
- `HC-05` 블루투스 모듈
- 18650 3.6V 배터리 x4 
### 3. OS, API
- FreeRTOS CMSIS-RTOS V2
- ssd1306 library
<img width = "30%" src="https://github.com/HeadlessJohn/RC_CAR_JOYSTICK/assets/159260193/7bcffdd5-c008-47f0-aae7-a053b6ceb888"> 
<img width = "30%" src="https://github.com/HeadlessJohn/RC_CAR_JOYSTICK/assets/159260193/33dd6dcd-93c7-48d1-9bbf-7fcc16ee1d2b"> 
<img width = "30%" src="https://github.com/HeadlessJohn/RC_CAR_JOYSTICK/assets/159260193/e37ecfc1-0f7b-497d-b154-31012fb6d8ba"> 

# RC Car Project
## Nucleo F411RE 보드를 이용한 RC Car
### 1. 기능
- 블루투스로 연결된 조이스틱으로부터 ADC 값 수신 [`RC_CAR_Controller`](https://github.com/HeadlessJohn/RC_CAR_CONTROLLER)
- 수신된 값을 계산하여 좌륜, 우륜 속도 분배
- LCD에 상태 표시
### 2. 사용한 부품
- ssd1306 I2C OLED 모듈
- DC모터 4개
- `L298N` 모터 드라이버
- `HC-05` 블루투스 모듈
- 18650 3.6V 배터리 x4 
### 3. OS, API
- FreeRTOS CMSIS-RTOS V2
- ssd1306 library
### 4. 주행 영상
- https://youtu.be/MAADGU-7uBE
- https://youtu.be/sVET32UEgNk
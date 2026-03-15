import time
import Jetson.GPIO as GPIO

# BOARD = 물리 핀 번호 (Jetson 40핀 헤더 기준)
PUL = 18   # 물리 18번핀을 예시로 사용 (원하면 바꿔도 됨)
DIR = 16   # 물리 16번핀 예시

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PUL, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(DIR, GPIO.OUT, initial=GPIO.LOW)

# 싱크 방식(ULN/NPN)에서는:
# GPIO HIGH -> 트랜지스터 ON -> PUL-가 GND로 당겨짐(펄스 "ON")
# GPIO LOW  -> 트랜지스터 OFF -> PUL- 해제(펄스 "OFF")
PULSE_ON  = GPIO.HIGH
PULSE_OFF = GPIO.LOW

def pulse(n, delay=0.001):
    for _ in range(n):
        GPIO.output(PUL, PULSE_ON)
        time.sleep(delay)
        GPIO.output(PUL, PULSE_OFF)
        time.sleep(delay)

try:
    # 정방향
    GPIO.output(DIR, GPIO.LOW)
    pulse(1000, delay=0.0008)
    time.sleep(0.3)

    # 역방향
    GPIO.output(DIR, GPIO.HIGH)
    pulse(1000, delay=0.0008)

finally:
    GPIO.cleanup()

import time
import Jetson.GPIO as GPIO

# ===================== 고정 핀(BOARD=물리핀) =====================
DIR_PIN = 33   # DIR- -> 물리핀 33
PUL_PIN = 37   # PUL- -> 물리핀 37
# EN은 일단 사용 안 함(테스트 안정)
USE_EN = False
EN_PIN = None

# ===================== 배선 방식 =====================
# direct: TB6600 PUL-/DIR-를 Jetson GPIO에 직결 (PUL+/DIR+는 +5V에 묶기)
# sink  : ULN2803/NPN으로 싱크 구동
WIRING_MODE = "direct"

# ===================== 기구값/동작 조건 =====================
STEPS_PER_MM = 400.0     # 너 기구에 맞게(예시)
MOVE_MM  = 40.0          # 4cm
REPEAT_N = 10
WAIT_SEC = 10.0          # 각 구간마다 10초 정지
STEP_FREQ = 800          # 처음엔 600~900 추천(안정 확인 후 1200+)

# =====================================================
def pulse_levels():
    # 직결(common-anode)일 때는 보통 PUL-을 LOW로 당길 때 옵토 ON
    if WIRING_MODE == "direct":
        return GPIO.LOW, GPIO.HIGH   # ON=LOW, OFF=HIGH
    else:
        return GPIO.HIGH, GPIO.LOW   # ON=HIGH, OFF=LOW

P_ON, P_OFF = pulse_levels()

def pulse_steps(n_steps: int):
    delay = 1.0 / (2.0 * STEP_FREQ)
    for _ in range(n_steps):
        GPIO.output(PUL_PIN, P_ON);  time.sleep(delay)
        GPIO.output(PUL_PIN, P_OFF); time.sleep(delay)

def move_mm(mm: float):
    GPIO.output(DIR_PIN, GPIO.HIGH if mm >= 0 else GPIO.LOW)
    steps = int(abs(mm) * STEPS_PER_MM)
    print(f"move {mm:.1f}mm -> {steps} steps", flush=True)
    pulse_steps(steps)

def main():
    GPIO.setmode(GPIO.BOARD)

    # direct 모드면 OFF=HIGH가 보통 안정(옵토 LED OFF)
    GPIO.setup(PUL_PIN, GPIO.OUT, initial=GPIO.HIGH if WIRING_MODE=="direct" else GPIO.LOW)
    GPIO.setup(DIR_PIN, GPIO.OUT, initial=GPIO.LOW)

    try:
        print(f"Start: {MOVE_MM}mm x {REPEAT_N}, wait {WAIT_SEC}s each | mode={WIRING_MODE}", flush=True)
        for i in range(REPEAT_N):
            print(f"[{i+1}/{REPEAT_N}] move +{MOVE_MM}mm", flush=True)
            move_mm(MOVE_MM)
            print(f"wait {WAIT_SEC}s...", flush=True)
            time.sleep(WAIT_SEC)
        print("Done", flush=True)

    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()

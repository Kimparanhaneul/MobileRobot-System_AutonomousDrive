#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import I2CDevice
from mindsensorsPYB import LSA
from pybricks.iodevices import UARTDevice
import math

# 이 프로그램은 LEGO EV3 MicroPython v2.0 이상에서 동작합니다.
th = 29.5
Gain = 5.5

# 노란색 RGB 기준값
yellow_r_min = 30
yellow_r_max = 49
yellow_g_min = 38
yellow_g_max = 56
yellow_b_min = 27
yellow_b_max = 60

# 빨간색 RGB 기준값
red_r_min = 27
red_g_max = 35
red_b_max = 35

# 주차 거리 제어를 위한 바퀴 회전 각도 변환 상수
WHEEL_DIAMETER_CM = 5.6
DEGREES_PER_CM = 360 / (math.pi * WHEEL_DIAMETER_CM)

# 하드웨어 초기화
ev3            = EV3Brick()
ser            = UARTDevice(Port.S2, baudrate=115200)
lsa            = LSA(Port.S4, 0x14)
run_motor      = Motor(Port.A)
steering_motor = Motor(Port.B)
l_ultra        = UltrasonicSensor(Port.S1)
color_sensor   = ColorSensor(Port.S3)

# 상태 변수 초기화
timer = StopWatch()
parking_detected = False
parking_done = False
pillar_count = 0
prev_in_range = False
count_yellow = 0
count_red    = 0
use_line_idx7 = False    # 라인 팔로잉 시 7번 인덱스 사용 여부
is_stopped_at_red_line = False # 3번째 빨간 선에서 멈췄는지 확인하는 플래그

ev3.speaker.beep()

def start():
    steering_motor.run(100)
    while True:
        a = steering_motor.angle()
        wait(100)
        b = steering_motor.angle()
        if a == b:
            break
    steering_motor.stop()
    steering_motor.run_angle(-100, 100)
    steering_motor.reset_angle(0)

def read_openmv_data():
    try:
        data = ser.read_all()
        if data:
            text = data.decode().strip().lower()[-5:]
            if text == 'reded':
                wait(500)
                return 'reded'
            elif text == 'green':
                wait(500)
                return 'green'
    except:
        pass
    return None


# 초기 캘리브레이션 & 전진
start()

# 2) 중앙 버튼 누를 때까지 대기
ev3.screen.print("Press CENTER to start")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
ev3.screen.clear()

# 3) 버튼 클릭 직후 주행 시작
run_motor.run(230)

while True:
    
    try:
        r, g, b = color_sensor.rgb()
        print("RGB:", r, g, b)


        # === 주차 공간 인식 ===
        if not parking_done:
            # (주차 로직은 베이스 코드와 동일하므로 생략)
            dist = l_ultra.distance()
            in_range = (dist <= 35)
            if in_range and not prev_in_range:
                pillar_count += 1
                ev3.speaker.beep()
                if pillar_count == 1:
                    timer.reset()
                if pillar_count == 2 and timer.time() >= 2000:
                    parking_detected = True
                if pillar_count == 3:
                    parking_detected = True
            prev_in_range = in_range

            if parking_detected:
                run_motor.stop()
                steering_motor.stop()
                steering_motor.run_target(1500, 0)
                wait(500)
                run_motor.run_angle(-200, int(13 * DEGREES_PER_CM))
                steering_motor.run_target(1500, -90)
                wait(500)
                run_motor.run_angle(200, int(10 * DEGREES_PER_CM))
                steering_motor.run_target(1500, 90)
                wait(500)
                run_motor.run_angle(-200, int(17 * DEGREES_PER_CM))
                steering_motor.run_target(1500, -90)
                wait(500)
                run_motor.run_angle(-200, int(19 * DEGREES_PER_CM))
                steering_motor.run_target(1500, 90)
                wait(500)
                run_motor.run_angle(200, int(6 * DEGREES_PER_CM))
                run_motor.stop()
                steering_motor.stop()
                ev3.speaker.beep()
                ev3.light.on(Color.RED)
                wait(1000)
                ev3.light.on(Color.ORANGE)
                wait(1000)
                ev3.light.on(Color.GREEN)
                wait(1000)
                ev3.light.off()
                steering_motor.run_target(1500, 0)
                wait(500)
                run_motor.run_angle(-200, int(6.7 * DEGREES_PER_CM))

                steering_motor.run_target(1500, -90)
                wait(500)
                run_motor.run_angle(200, int(18 * DEGREES_PER_CM))

                steering_motor.run_target(1500, 72)
                wait(500)
                run_motor.run_angle(200, int(21 * DEGREES_PER_CM))
                steering_motor.run_target(1500, 0)
                run_motor.run(230)
                use_line_idx7 = False
                parking_done   = True

        # === 색상 센서 처리 ===
        # ======================= 코드 수정 시작 (1/3) =======================
        # is_stopped_at_red_line 플래그가 False일 때만 빨간색을 감지
        if not is_stopped_at_red_line and r >= red_r_min and g <= red_g_max and b <= red_b_max:
            count_red += 1
            if count_red == 1:
                ev3.speaker.beep()
                wait(500)
            elif count_red == 2:
                ev3.speaker.beep()
                wait(500)
            elif count_red == 3:
                ev3.speaker.beep()
                run_motor.stop()
                steering_motor.stop()
                
                # ======================= 코드 수정 시작 (2/3) =======================
                # 3번째 빨간 선에 멈췄으므로 플래그를 True로 설정
                is_stopped_at_red_line = True
                # ======================= 코드 수정 끝 (2/3) =======================

                consecutive_green_count = 0
                while consecutive_green_count < 2:
                    label = read_openmv_data()
                    print("Waiting for green signal. Received:", label)

                    if label == 'green':
                        consecutive_green_count += 1
                    else:
                        consecutive_green_count = 0
                    
                    wait(100)

                print("Two consecutive green signals received. Restarting.")
                ev3.speaker.beep()
                ev3.speaker.beep()
                run_motor.run(230)
                
                # ======================= 코드 수정 시작 (3/3) =======================
                # 로봇이 움직여서 현재의 빨간 선을 벗어날 때까지 기다림
                while True:
                    r_check, g_check, b_check = color_sensor.rgb()
                    if not (r_check >= red_r_min and g_check <= red_g_max and b_check <= red_b_max):
                        # 더 이상 빨간색이 아니면, 플래그를 초기화하고 루프 탈출
                        is_stopped_at_red_line = False
                        print("Off the 3rd red line. Red line detection re-enabled.")
                        break
                    wait(100) # CPU 부하 감소를 위한 짧은 대기
                # ======================= 코드 수정 끝 (3/3) =======================
                continue

            elif count_red == 4:
                run_motor.stop()
                steering_motor.stop()
                for _ in range(3):
                    ev3.speaker.beep()
                    wait(100)
                break
        # ======================= 코드 수정 끝 (1/3) =======================

        # 노란색 감지 (기존 처리)
        if count_yellow == 0 and \
           yellow_r_min <= r <= yellow_r_max and \
           yellow_g_min <= g <= yellow_g_max and \
           yellow_b_min <= b <= yellow_b_max:
            run_motor.stop()
            steering_motor.stop()
            ev3.speaker.beep()
            ev3.light.on(Color.RED)
            wait(1000)
            ev3.light.on(Color.ORANGE)
            wait(1000)
            ev3.light.on(Color.GREEN)
            wait(1000)
            ev3.light.off()
            count_yellow += 1
            use_line_idx7 = False
            run_motor.run(230)
            continue

        # === 라인 팔로잉 제어 (인덱스 6 또는 7 사용) ===
        data = lsa.ReadRaw_Calibrated()
        idx = 7 if use_line_idx7 else 6
        line_value = list(data)[idx]
        error = th - line_value
        correction = max(min(Gain * error, 90), -90)
        steering_motor.run_target(1500, correction)

    except:
        pass
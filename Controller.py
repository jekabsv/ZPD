from time import sleep_us, ticks_us, ticks_ms, sleep
import machine
from machine import Pin, PWM
import math
from PID import PID
from HAMA import HAMA
import sys, select, time

MAX_CS = 11
MAX_SO = 12
MAX_SCK = 10
cs = Pin(MAX_CS, Pin.OUT, value=1)
so = Pin(MAX_SO, Pin.IN)
sck = Pin(MAX_SCK, Pin.OUT, value=0)

def read_raw():
    cs.value(0)
    sleep_us(1)
    value = 0
    for _ in range(32):
        sck.value(1)
        value = (value << 1) | so.value()
        sck.value(0)
    cs.value(1)
    return value

def decode_max31855(data):
    if data & 0x00010000:
        return None, None, True
    tc = (data >> 18) & 0x3FFF
    if tc & 0x2000:
        tc -= 0x4000
    tc_temp = tc * 0.25
    cj = (data >> 4) & 0xFFF
    if cj & 0x800:
        cj -= 0x1000
    cj_temp = cj * 0.0625
    return tc_temp, cj_temp, False

HEATER_PIN = 15
heater_pwm = PWM(Pin(HEATER_PIN))
heater_pwm.freq(9)
heater_pwm.duty_u16(0)

Kp = 6000
Ki = 0
Kd = 18000
pid = PID(Kp, Ki, Kd, output_limits=(0, 65535), scale='s')

hama = HAMA(26, 22, 21)
defexposure = 0.01

StartTemp = 130
StopTemp = 150
TempSpeed = 5
tolerance = 5
running = False
next_spectrum_temp = StartTemp
pid_start_time = None
process_time = 0
prev_time = ticks_ms()
DEBUG_AUTO_START = False
auto_started = False
duty = 0

while True:
    raw = read_raw()
    tc, cj, fault = decode_max31855(raw)
    temp = float("nan") if fault else tc
    temp_str = "NaN" if math.isnan(temp) else f"{temp:.2f}"

    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        line = sys.stdin.readline().strip()
        if line:
            try:
                numbers = list(map(float, line.replace(',', '.').split()))
                cmd = int(numbers[0])
                if cmd == 0:
                    pid.output_sum = 0
                    pid.last_input = 0
                    pid.deactivated = False
                    last_spectrum_time = time.ticks_ms()
                    StartTemp = 0
                    StopTemp = 0
                    TempSpeed = 0
                    current_setpoint = 0
                    process_time = 0
                    prev_spectrum_temp = 0
                    pid_start_time = None
                    next_spectrum_temp = 0
                    running = False
                    auto_started = False
                    tolerance = 0.5
                    heater_pwm.duty_u16(0)
                elif cmd == 1:
                    TempSpeed = numbers[1]
                    StartTemp = numbers[2]
                    StopTemp = numbers[3]
                    tolerance = numbers[4] if len(numbers) > 4 else 0.5
                    running = True
                    pid_start_time = time.ticks_ms()
                    process_time = 0
                    prev_spectrum_temp = StartTemp
                    current_setpoint = StartTemp
                    next_spectrum_temp = StartTemp
                    auto_started = True
            except ValueError:
                pass
            
            
    if DEBUG_AUTO_START and not auto_started:
        running = True
        pid_start_time = ticks_ms()
        process_time = 0
        next_spectrum_temp = StartTemp
        pid.setpoint = StartTemp
        auto_started = True
        prev_time = ticks_ms()

    if running and not math.isnan(temp):
        dt = ticks_ms() - prev_time
        prev_time = ticks_ms()
        if process_time is not None:
            process_time += dt / 60000
            T_set_pid = StartTemp + TempSpeed * process_time
            if T_set_pid > next_spectrum_temp:
                T_set_pid = next_spectrum_temp
                process_time -= dt / 60000
        else:
            T_set_pid = pid.setpoint
        pid.setpoint = T_set_pid
        pid_output = pid(temp)
        duty = int(pid_output)
        heater_pwm.duty_u16(duty)

        if abs(temp - next_spectrum_temp) <= tolerance:
#             overexposed, defexposure, spectra = hama.autoexposure(defexposure)
#             spectrum_str = ",".join(str(v) for v in spectra)
#             print(f"Spectrum at Temp:{temp_str}C | Data:[{spectrum_str}]")
#             if overexposed:
#                 print("Probably overexposed!")
            spectra = hama.main(1, 1)
            spectrum_str = ",".join(str(v) for v in spectra)
            print(f"Spectrum at Temp:{temp_str}C | Data:[{spectrum_str}]")
            next_spectrum_temp += 2
            if next_spectrum_temp > StopTemp:
                next_spectrum_temp = StopTemp
                process_time = 0

        if temp >= StopTemp-tolerance:
            heater_pwm.duty_u16(0)
            running = False
            process_time = 0
            T_set_pid = 0
            pid.setpoint = T_set_pid

    print(f"Temp:{temp_str}C | Next setpoint:{next_spectrum_temp:.2f}C | PID setpoint:{pid.setpoint:.2f}C | duty:{duty}")
    sleep(0.5)


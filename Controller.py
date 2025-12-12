from time import sleep_us, ticks_us, ticks_ms, sleep
import machine
from machine import Pin, PWM
import math
from PID import PID
import sys, select, time

class HAMA:
    def __init__(self, pin_adc=26, pin_clk=22, pin_st=21):
        self.picoADC = machine.ADC(pin_adc)
        self.CLK = pin_clk
        self.ST = pin_st
        self.pinST = Pin(self.ST, Pin.OUT, Pin.PULL_DOWN)
        sleep_us(10)
        self.spectra = []
        self.delay = 1
        self.accum = 1
        self.exposure = 0.001

    def repeater(self, times, f, *args):
        for _ in range(times):
            f(*args)

    def sample_spec(self):
        target = ticks_us() + int(self.exposure * 1e6)
        while ticks_us() < target:
            pass
        self.PWMCLK.duty_u16(0)
        self.PWMCLK.deinit()

    def read_spec(self):
        self.pinCLK.high()
        sleep_us(self.delay)
        self.spectra.append(self.picoADC.read_u16())
        self.pinCLK.low()
        sleep_us(self.delay)

    def cycle_spec(self):
        self.pinCLK.high()
        self.pinCLK.low()

    def start(self):
        self.PWMCLK = PWM(Pin(self.CLK))
        self.PWMCLK.freq(125000)
        self.PWMCLK.duty_u16(32768)
        sleep_us(self.delay)
        self.pinST.high()
        self.sample_spec()
        self.pinST.low()
        self.pinCLK = Pin(self.CLK, Pin.OUT, Pin.PULL_DOWN)
        self.repeater(48, self.cycle_spec)
        self.repeater(39, self.cycle_spec)
        self.repeater(288, self.read_spec)
        self.repeater(2, self.cycle_spec)
        sleep_us(self.delay)
        return self.spectra

    def acquisition(self):
        self.spectra = []
        return self.start()

    def main(self, exposure, accum):
        self.accum = int(round(accum))
        self.exposure = exposure
        self.spectra = self.acquisition()
        if self.accum > 1:
            acc = self.spectra
            for _ in range(self.accum - 1):
                tmp = self.acquisition()
                acc = [a + b for a, b in zip(acc, tmp)]
            return acc
        return self.spectra

    def autoexposure(self, exposure):
        self.exposure = exposure
        self.spectra = self.acquisition()
        done = False
        while not done:
            mx = max(self.spectra)
            if 45000 <= mx <= 65530:
                done = True
            elif mx > 65530 and self.exposure < 2:
                if self.exposure > 1e-9:
                    self.exposure /= 2
                    self.spectra = self.acquisition()
                else:
                    done = True
            elif mx < 14000 and self.exposure < 0.33:
                if self.exposure < 0.04:
                    self.exposure *= 10
                else:
                    self.exposure *= 2
                self.spectra = self.acquisition()
            elif 14000 <= mx <= 45000 and self.exposure < 0.33:
                self.exposure = self.exposure * 55000 / mx
                self.spectra = self.acquisition()
            else:
                done = True
        return False, self.exposure, self.spectra

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

Kp = 6500
Ki = 0
Kd = 18000
pid = PID(Kp, Ki, Kd, output_limits=(0, 65535), scale='s')

hama = HAMA(27, 22, 21)
defexposure = 0.01

StartTemp = 50
StopTemp = 55
TempSpeed = 1
tolerance = 0.25
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
            overexposed, defexposure, spectra = hama.autoexposure(defexposure)
            spectrum_str = ",".join(str(v) for v in spectra)
            print(f"Spectrum at Temp:{temp_str}C | Data:[{spectrum_str}]")
            next_spectrum_temp += TempSpeed
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


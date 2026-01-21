from time import sleep_us, ticks_us, ticks_ms
import machine
from machine import Pin, PWM

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
        start = ticks_us()
        end = start + self.exposure * 1e6
        while ticks_us() < end:
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
        self.PWMCLK.freq(125_000_000 // 1000)
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
        self.exposure = exposure
        self.accum = int(accum)

        base = self.acquisition()

        if self.accum > 1:
            total = base[:]
            for _ in range(self.accum - 1):
                s2 = self.acquisition()
                total = [a + b for a, b in zip(total, s2)]
            return total
        return base

    def autoexposure(self, exposure):
        self.exposure = exposure
        self.spectra = self.acquisition()

        thisisgood = False
        overexposed = False

        while not thisisgood:
            mx = max(self.spectra)

            if 45000 <= mx <= 65530:
                thisisgood = True

            elif mx > 65530 and self.exposure < 2:
                if self.exposure > 1e-9:
                    self.exposure /= 2
                    self.spectra = self.acquisition()
                else:
                    thisisgood = True
                    overexposed = True

            elif mx < 14000 and self.exposure < 0.33:
                if self.exposure < 0.04:
                    self.exposure *= 10
                else:
                    self.exposure *= 2
                self.spectra = self.acquisition()

            elif 14000 <= mx < 45000 and self.exposure < 0.33:
                self.exposure = self.exposure * 55000 / mx
                self.spectra = self.acquisition()

            else:
                thisisgood = True

        return overexposed, self.exposure, self.spectra


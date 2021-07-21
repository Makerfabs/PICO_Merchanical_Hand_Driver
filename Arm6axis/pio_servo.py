from rp2 import PIO, StateMachine, asm_pio
from machine import Pin
import time

T_X = 4000
T_Y0 = 3900
T_Y180 = 3500


#freq=800000 0.5ms y=3900 2.5ms y=3500 unit_y:2.5us == 0.45degree
@asm_pio(set_init=PIO.OUT_LOW)
def servo_ctrl2():

    mov(x, isr)
    set(pins, 1)
    
    label("for")
    jmp(not_x,"over")
    jmp(x_not_y,"dec")
    set(pins, 0)
    label("dec")
    jmp(x_dec,"continue")
    label("continue")
    jmp("for")

    label("over")
    nop()[3]


class PIO_SERVO:
    def __init__(self, sm_id, pin):
        self._sm = StateMachine(
            sm_id, servo_ctrl2, freq=800000, set_base=Pin(pin))
        # Use exec() to load max count into ISR
        #self._sm.put(T_Y0)
        self._sm.put(T_X)
        self._sm.exec("pull()")
        self._sm.exec("mov(y, osr)")
        self._sm.put(T_X)
        self._sm.exec("pull()")
        self._sm.exec("mov(isr, osr)")
        self._sm.active(1)

    def set(self, pio_cycle):
        self._sm.active(0)
        self._sm.put(pio_cycle)
        self._sm.exec("pull()")
        self._sm.exec("mov(y, osr)")
        self._sm.put(T_X)
        self._sm.exec("pull()")
        self._sm.exec("mov(isr, osr)")
        self._sm.active(1)

    def set_ms(self, ms):
        if ms < 0.5:
            ms = 0.5
        elif ms > 2.5:
            ms = 2.5

        pio_cycle = int(3900 - (ms - 0.5) * 200)
        self.set(pio_cycle)
    
    def set_angle(self, angle):
        ms = (angle + 90.0) / 90.0 + 0.5
        self.set_ms(ms)


def test():

    servo = PIO_SERVO(0, 16)
    # servo.set(180)
    # servo.set_ms(1.5)
    while 0:
        time.sleep(3)
    while 1:

        # servo.set(T_Y180)
        servo.set_ms(0.5)

        time.sleep(3)

        # servo.set(T_Y0)
        servo.set_ms(2.5)

        time.sleep(3)


#test()

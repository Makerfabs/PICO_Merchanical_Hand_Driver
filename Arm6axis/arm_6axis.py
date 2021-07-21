from pio_servo import PIO_SERVO
import time

# PIN_LIST = [0, 1, 2, 3, 4, 5]
# DIR_LIST = [0, 0, 0, 1, 0, 0]


class Arm_axis:
    def __init__(self, index, pin, direct):
        self.id = index
        self.pin = pin
        self.dir = direct
        # default position is middle
        self.angle = 0.0
        self.servo = PIO_SERVO(self.id, self.pin)

    def set_angle(self, angle):
        if angle < -90.0:
            angle = -90.0
        elif angle > 90.0:
            angle = 90.0
        self.angle = angle

    def fresh(self):
        real_angle = self.angle
        if self.dir == 1:
            real_angle = -real_angle
        self.servo.set_angle(real_angle)

    def angle_report(self):
        return self.angle


class Arm_6:
    def __init__(self, pin_list, dir_list):
        self.axises = []
        for i in range(6):
            self.axises.append(
                Arm_axis(i, pin_list[i], dir_list[i]))

    def flesh_single(self, index):
        self.axises[index].fresh()

    def flesh(self):
        for i in range(6):
            self.axises[i].fresh()

    def set_single_angle(self, index, angle):
        self.axises[index].set_angle(angle)

    def pos_report(self):
        pos = []
        for i in range(6):
            pos.append(self.axises[i].angle_report())

        return pos

def test():
    PIN_LIST = [0, 1, 2, 3, 4, 5]
    DIR_LIST = [0, 0, 0, 1, 0, 0]

    arm = Arm_6(PIN_LIST, DIR_LIST)
    time.sleep(3)

    while 1:

        #arm.set_single_ms(3, 2.5)
        arm.set_single_angle(3, -90)
        arm.flesh()
        print("-90")
        time.sleep(3)

        arm.set_single_angle(3, 90)
        arm.flesh()
        print("90")
        time.sleep(3)

from arm_6axis import Arm_6
import time
import machine

RUNFILE = "arm_code7.txt"
DEBUG = 0

# PIN_LIST = [16, 17, 18, 19, 20, 21]
# DIR_LIST = [0, 0, 0, 1, 0, 0]
# ANGLE_LIST = [-2, -25, 75, 75, 0, 40]

PIN_LIST = [16, 17, 18, 19, 20, 21]
DIR_LIST = [0, 1, 1, 0, 0, 1]
ANGLE_LIST = [-2, -40, 75, 75, 0, -10]


DELAY_TIME = 0.5


class Arm_controller:
    def __init__(self, PIN_LIST, DIR_LIST):
        self.arm = Arm_6(PIN_LIST, DIR_LIST)

    def explain_line(self, TEXT):
        if TEXT[0] == "#":
            if DEBUG:
                print(TEXT)
            return 0

        elif TEXT.find("ABS") == 0:
            command_list = TEXT[4:].split(" ")
            if DEBUG:
                print("ABS")
                print(command_list)
            self.explain_absolate(command_list)
            return 1

        elif TEXT.find("REL") == 0:
            command_list = TEXT[4:].split(" ")
            if DEBUG:
                print("REL")
                print(command_list)
            self.explain_relative(command_list)
            return 1

        elif TEXT.find("END") == 0:
            print("COMMAND END")
            print("Finally POS:")
            print(self.arm.pos_report())
            return 2

        elif TEXT.find("POS") == 0:
            self.arm.flesh()
            time.sleep(2)
            print("REALLY POS:")
            print(self.arm.pos_report())
            return 1

        elif TEXT.find("WAIT") == 0:
            time.sleep(2)
            return 1

        elif TEXT.find("SLEEP") == 0:
            sleep_time = int(TEXT[6:])
            print("Sleep " + str(sleep_time) + "...")
            time.sleep(sleep_time)
            return 1

        elif TEXT.find("BUZZ") == 0:
            alarm()
            return 1

        elif TEXT.find("BUTT") == 0:
            input_button()
            return 1

        else:
            print("WARNING : NOT KNOWN COMMAND")
            print(TEXT)
            return -1

    def explain_absolate(self, COMMAND_LIST):
        for part in COMMAND_LIST:
            index = self.get_index(part[0])
            angle = int(part[1:])

            # print(index)
            # print(angle)

            self.arm.set_single_angle(index, angle)
        self.arm.flesh()
        time.sleep(DELAY_TIME)

    def explain_relative(self, COMMAND_LIST):
        pos_list = self.arm.pos_report()
        for part in COMMAND_LIST:
            index = self.get_index(part[0])
            angle = int(part[1:]) + pos_list[index]

            # print(index)
            # print(angle)

            self.arm.set_single_angle(index, angle)
        self.arm.flesh()
        time.sleep(DELAY_TIME)

    # init by angle list
    def init_arm_angle(self, ANGLE_LIST):
        for index in range(6):
            self.arm.set_single_angle(index, ANGLE_LIST[index])
        self.arm.flesh()
        time.sleep(2)

    def get_index(self, part):
        axis_dic = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5}
        return axis_dic[part]


def control_by_file():

    ct = Arm_controller(PIN_LIST, DIR_LIST)
    ct.init_arm_angle(ANGLE_LIST)

    file = open(RUNFILE, "r")

    for line in file:
        print(line)
        if line.isspace():
            continue
        if ct.explain_line(line) == 2:
            break
    file.close()


def control_by_serial():
    ct = Arm_controller(PIN_LIST, DIR_LIST)
    ct.init_arm_angle(ANGLE_LIST)
    line = ""
    while 1:
        line = input()
        if line.isspace():
            continue
        if ct.explain_line(line) == 2:
            break
        line = ""


def control_by_serial_save():
    file = open("Log.txt", "w")
    file.write("#" + str(PIN_LIST) + "\n")
    file.write("#" + str(DIR_LIST) + "\n")
    file.write("#" + str(ANGLE_LIST) + "\n")

    ct = Arm_controller(PIN_LIST, DIR_LIST)
    ct.init_arm_angle(ANGLE_LIST)
    line = ""
    while 1:
        line = input()
        file.write(str(line) + "\n")
        if line.isspace():
            continue
        if ct.explain_line(line) == 2:
            break
        line = ""

    file.close()


def control_for_factory_test():

    dir_list_factory = [0, 0, 0, 0, 0, 0]
    angle_list_factory = [0, 0, 0, 0, 0, 0]
    ct = Arm_controller(PIN_LIST, dir_list_factory)
    ct.init_arm_angle(angle_list_factory)
    ct.explain_line("BUZZ")
    while 1:
        ct.explain_line("BUTT")
        ct.explain_line("BUZZ")
        ct.explain_line("ABS A45 B45 C45 D45 E45 F45")
        ct.explain_line("WAIT")
        ct.explain_line("ABS A-45 B-45 C-45 D-45 E-45 F-45")
        # ct.explain_line("WAIT")


def alarm():
    buzzer = machine.Pin(15, machine.Pin.OUT)
    for i in range(500):
        buzzer.value(1)
        time.sleep(0.001)
        buzzer.value(0)
        time.sleep(0.001)


def input_button():
    button = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
    print("Wait button press")
    while 1:
        if button.value() == 0:
            time.sleep(0.5)
            if button.value() == 0:
                alarm()
                break
        time.sleep(0.5)


def test():

    # while 1:
    #     control_by_file()
    #     break
    # control_by_serial()
    # control_by_serial_save()
    control_for_factory_test()


test()

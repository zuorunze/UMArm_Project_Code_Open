import serial
import time


def arm_stop():
    rs485_pc_terminal = serial.Serial(port='COM37', baudrate=115200, timeout=.1)


    str_to_be_sent = "bb0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_pc_terminal.write(utf8str)


    time.sleep(2)

    str_to_be_sent = "ex0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_pc_terminal.write(utf8str)


    time.sleep(6)

    str_to_be_sent = "ss0 9999"
    utf8str = str_to_be_sent.encode('utf-8') + b'\n'
    rs485_pc_terminal.write(utf8str)


if __name__ == "__main__":
    arm_stop()

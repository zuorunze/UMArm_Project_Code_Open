import serial,time
import numpy as np
import robot_constants as rc

def mp_direct_p_control(input_mp_p_actuator, output_actuator_pressure, input_mp_control_flag):
    pctrl = direct_p_control(input_mp_p_actuator, output_actuator_pressure, input_mp_control_flag)

    pctrl.direct_p_control_main()

    return


def mp_p_controller(mp_p_actuator):
    while True:
        p_target = np.array([5,6,7,8,  9,10,11,12,
                             5,5,5,5,  5,5,5,5,
                             5,5,5,5,  5,5,5,5])
        
        for i in range(len(mp_p_actuator)):
            mp_p_actuator[i] = p_target[i]

        time.sleep(5)
    return

class direct_p_control:
    mp_p_actuator = None
    mp_p_output = None
    mp_control_flag = None

    rs_485_interface = None
    p_desired_global = None

    serial_freq = 25.0
    
    
    def __init__(self, input_mp_p_actuator, output_actuator_pressure, input_mp_control_flag):
        self.mp_p_actuator     = input_mp_p_actuator
        self.mp_p_output       = output_actuator_pressure
        self.mp_control_flag   = input_mp_control_flag


    def direct_p_control_main(self):
        try: # try connecting to the port
            while self.initialize_serial(rc.COM_port,rc.BAUD_RATE) is False:
                time.sleep(1)
                print("trying serial connection again......") 
        except Exception as e:
            print("serial connection:", e)

        try: # try running the pressure controller
            serial_stopped = True
            while True:
                if self.mp_control_flag[rc.direct_p_control] == 1:
                    serial_stopped = False
                    self.send_desired_p_to_arm()
                else:
                    if serial_stopped == False:
                        self.arm_stop()
                        serial_stopped = True

                time.sleep(1.0/self.serial_freq)

        except Exception as e:
            print("direct_p_control: ", e)

    def arm_stop(self):
        str_to_be_sent = "bb0 9999"
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        self.rs_485_interface.write(utf8str)

        time.sleep(0.2)
        str_to_be_sent = "ex0 9999"
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        self.rs_485_interface.write(utf8str)

        time.sleep(5)

        str_to_be_sent = "ss0 9999"
        utf8str = str_to_be_sent.encode('utf-8') + b'\n'
        self.rs_485_interface.write(utf8str)
        time.sleep(0.2)
        print("NOTE: arm_stopped")

    def initialize_serial(self,com,baud):
        try:
            print("--Connecting to {}".format(com))
            self.rs_485_interface = serial.Serial(port=com, baudrate=baud, timeout=.1)
        except:
            print("--Connection failed")
            return False
        else:
            print("--Connected")
        return True


    def send_desired_p_to_arm(self):
        # update the actuator pressure to the arm
        for i, act_addr in enumerate(rc.actuator_address_list):
            act_pressure = self.mp_p_actuator[i]
            # also update the actuator pressure to the mp variable
            self.mp_p_output[i] = act_pressure * 6894.76 # psi to pa

            str_to_be_sent = "bb{:.2f} {}".format(act_pressure, int(act_addr))
            utf8str = str_to_be_sent.encode('utf-8') + b'\n'
            self.rs_485_interface.write(utf8str)
        
        return
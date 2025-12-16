import time
from servo import Servo
from control import Control
from command import COMMAND as cmd
from adc import ADC
import sys

def main():
    adc = ADC()
    servo = Servo()
    ctrl = Control()
    ctrl.condition_thread.start()


    try:
        ctrl.command_queue = [cmd.CMD_POSITION, "0", "0", "10"]
        time.sleep(2.0)
        for _ in range(5):
            ctrl.run_gait([cmd.CMD_MOVE, "2", "0", "20", "8", "0"])
            time.sleep(0.005)
            ctrl.command_queue = [cmd.CMD_POSITION, "0", "0", "10"]
            time.sleep(0.005)
            Power=adc.read_battery_voltage()
            print ("The battery voltage is "+str(Power)+'\n')
        servo.relax()
        print ("\nEnd of program")
    except KeyboardInterrupt:
        servo.relax()
        print ("\nEnd of program")

if __name__ == '__main__':
    print ('Program is starting ... ')
    main()

import time
import math
import copy
import threading
import numpy as np
from gpiozero import OutputDevice
import RPi.GPIO as GPIO

from pnuematics import PnuematicsController
from transducer import Transducer
from servo import Servo
from control import Control
from command import COMMAND as cmd
from adc import ADC
import sys
from typing import Dict, List, Mapping, Optional, Tuple


# Control (src/Server/control.py) leg indices are:
#   0: front right, 1: back right, 2: back left, 3: front left
DEFAULT_LEG_TO_VALVE: Mapping[int, int] = {
    0: 2,  # front right -> valve 2
    1: 4,  # back right  -> valve 4
    2: 3,  # back left   -> valve 3
    3: 1,  # front left  -> valve 1
}

class StateMachine:
    def __init__(self):
        self.states = ['RELAXED', 'STATIONED', 'WALKING', 'CLIMBING']
        self.current_state = 'RELAXED'
        self.state_lock = threading.Lock()
        self.ctrl = Control()
        self.servo = Servo()
        self.pneumatics = PnuematicsController()
        self.transducer = Transducer()
        self.adc = ADC() # Battery voltage reader. Eventually add functionality to enter emergency mode on low battery.

        self.ctrl.condition_thread.start()

    def transition_to(self, new_state):
        with self.state_lock:
            if new_state in self.states:
                print(f"Transitioning from {self.current_state} to {new_state}")
                self.current_state = new_state
            else:
                print(f"Invalid state transition attempted: {new_state}")

    def get_current_state(self):
        with self.state_lock:
            return self.current_state
        
    def run_state_actions(self):
        while True:
            with self.state_lock:
                state = self.current_state
            
            if state == 'RELAXED':
                self.relaxed_actions()
            elif state == 'STATIONED':
                self.stationed_actions()
            elif state == 'WALKING':
                self.walking_actions()
            elif state == 'CLIMBING':
                self.climbing_actions()
            
            time.sleep(0.1)

    def relaxed_actions(self):
        # print("Entered RELAXED state.")
        # Add RELAXED state specific actions here
        if max(self.transducer.read_all_pressures()) > 0.5:
            self.pneumatics.close_all_valves()
            print("Releasing pressure to reach RELAXED state...")
        self.servo.relax()
        print("Robot is now RELAXED.")

    def stationed_actions(self):
        # print("Entered STATIONED state.")
        # Add STATIONED state specific actions here
        self.ctrl.command_queue = [cmd.CMD_POSITION, "0", "0", "10"]
        if min(self.transducer.read_all_pressures()) < 75.0:
            self.pneumatics.open_all_valves()
            print("Building pressure to reach STATIONED state...")
        print("Robot is now STATIONED.")

    def walking_actions(self):  
        # print("Entered WALKING state.")
        # Add WALKING state specific actions here
        self.ctrl.run_gait([cmd.CMD_MOVE, "2", "0", "20", "8", "0"])

    def climbing_actions(self):
        # print("Entered CLIMBING state.")
        # Add CLIMBING state specific actions here
        pass


if __name__ == '__main__':
    state_machine = StateMachine()

    # Background thread runs state actions
    state_thread = threading.Thread(
        target=state_machine.run_state_actions,
        daemon=True
    )
    state_thread.start()

    print("State machine started.")
    print("Type a state name (RELAXED, STATIONED, WALKING, CLIMBING) or 'q' to quit.")

    try:
        while True:
            user_cmd = input(">> ").strip().upper()

            if user_cmd in ('Q', 'QUIT', 'EXIT'):
                state_machine.relaxed_actions()  # Ensure safe state before exiting
                print("Exiting state machine.")
                break

            try:
                state_machine.transition_to(user_cmd)
            except ValueError as e:
                print(f"Invalid state: {user_cmd}")
                print("Valid states: RELAXED, STATIONED, WALKING, CLIMBING")

    except KeyboardInterrupt:
        print("\nExiting state machine.")

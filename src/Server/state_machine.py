import time
import math
import copy
import threading
import numpy as np
from gpiozero import OutputDevice
import RPi.GPIO as GPIO

# from pnuematics import PnuematicsController
# from transducer import Transducer
from servo import Servo
from control import Control
from command import COMMAND as cmd
from adc import ADC
import sys
from typing import Dict, List, Mapping, Optional, Tuple

STATE_MAP = {
    'R': 'RELAXED',
    'S': 'STATIONED',
    'W': 'WALKING',
    'C': 'CLIMBING',
}

class StateMachine:
    def __init__(self):
        self.states = ['RELAXED', 'STATIONED', 'WALKING', 'CLIMBING']
        self.current_state = 'RELAXED'
        self.last_state = None
        self.state_lock = threading.Lock()
        self.ctrl = Control()
        self.servo = Servo()
        # self.pneumatics = PnuematicsController()
        # self.transducer = Transducer()
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
            state = self.get_current_state()

            # Run "on-enter" actions once per transition
            if state != self.last_state:
                self.on_enter(state)
                self.last_state = state

            # Run actions 
            if state == 'RELAXED':
                self.relaxed_actions()
            elif state == 'STATIONED':
                self.stationed_actions()
            elif state == 'WALKING':
                self.walking_actions()
            elif state == 'CLIMBING':
                self.climbing_actions()

            time.sleep(0.1)

    def on_enter(self, state):
        print(f"\n[STATE] Entered {state}")
        
        if state == 'RELAXED':
            self.ctrl.pneumatics.close_all_valves()
        elif state == 'STATIONED':
            self.ctrl.pneumatics.open_all_valves()
            pass
        elif state == 'WALKING':
            pass
        elif state == 'CLIMBING':
            pass

        time.sleep(0.1)

    def relaxed_actions(self):
        self.servo.relax()

    def stationed_actions(self):
        self.ctrl.command_queue = [cmd.CMD_POSITION, "0", "0", "15"]

    def walking_actions(self):  
        self.ctrl.run_gait([cmd.CMD_MOVE, "2", "0", "20", "3", "0", "0"])

    def climbing_actions(self):
        self.ctrl.run_gait([cmd.CMD_MOVE, "2", "0", "20", "3", "0", "1"])
        
    

if __name__ == '__main__':
    state_machine = StateMachine()

    # Background thread runs state actions
    state_thread = threading.Thread(
        target=state_machine.run_state_actions,
        daemon=True
    )
    state_thread.start()

    print("State machine started.")
    print("Type a state name (r, s, w, c) or 'q' to quit.")

    try:
        while True:
            user_cmd = input(">> ").strip().upper()

            if user_cmd in ('Q', 'QUIT', 'EXIT'):
                state_machine.ctrl.pneumatics.close_all_valves()  # Ensure safe state before exiting
                state_machine.servo.relax()
                print("Exiting state machine.")
                break

            try:
                state_machine.transition_to(STATE_MAP[user_cmd])
            except ValueError as e:
                print(f"Invalid state: {user_cmd}")
                print("Valid states: r, s, w, c")

    except KeyboardInterrupt:
        state_machine.ctrl.pneumatics.close_all_valves()  # Ensure safe state before exiting
        state_machine.servo.relax()
        print("\nExiting state machine.")
        exit()


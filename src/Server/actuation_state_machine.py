# -*- coding: utf-8 -*-
"""Actuation state machine for gait + pneumatics.

This module is intentionally self-contained so it can be integrated incrementally.

Key ideas:
- Gait actuation is run in a dedicated thread (because Control.run_gait() is blocking).
- Pneumatic actuation can be held, pulsed, or sequenced.
- A simple state machine arbitrates between IDLE / GAIT / PNEUMATICS / COMBINED / ESTOP.

Typical integration point:
- In src/Server/server.py, construct ActuationStateMachine(Control(), PneumaticsController())
  and route incoming commands to sm.set_gait_command(...) / sm.set_pneumatic_...().
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Mapping, Optional, Tuple


DEFAULT_VALVE_PINS: Mapping[int, int] = {
    1: 13,  # Front Left
    2: 19,  # Front Right
    3: 16,  # Back Left
    4: 20,  # Back Right
}

# Control (src/Server/control.py) leg indices are:
#   0: front right, 1: back right, 2: back left, 3: front left
DEFAULT_LEG_TO_VALVE: Mapping[int, int] = {
    0: 2,  # front right -> valve 2
    1: 4,  # back right  -> valve 4
    2: 3,  # back left   -> valve 3
    3: 1,  # front left  -> valve 1
}


class ActuationState(str, Enum):
    IDLE = "IDLE"
    GAIT_ONLY = "GAIT_ONLY"
    PNEUMATICS_ONLY = "PNEUMATICS_ONLY"
    GAIT_AND_PNEUMATICS = "GAIT_AND_PNEUMATICS"
    ESTOP = "ESTOP"


class GaitPneumaticSyncMode(str, Enum):
    OFF = "OFF"
    SWING_OPEN = "SWING_OPEN"  # open valves for swing legs
    STANCE_OPEN = "STANCE_OPEN"  # open valves for stance legs


@dataclass(frozen=True)
class GaitCommand:
    """Mirrors the CMD_MOVE payload used by Control.run_gait().

    Expected format: ["CMD_MOVE", gait, x, y, speed, angle]
    where all fields are strings (as parsed from the socket protocol).
    """

    data: Tuple[str, str, str, str, str, str]

    @property
    def gait(self) -> str:
        return self.data[1]

    @property
    def speed(self) -> int:
        return int(self.data[4])


@dataclass(frozen=True)
class PneumaticStep:
    duration_s: float
    valve_states: Mapping[int, bool]


@dataclass(frozen=True)
class PneumaticCommand:
    """Pneumatic actuation request.

    Exactly one of (hold_states, sequence) should be provided.
    """

    hold_states: Optional[Mapping[int, bool]] = None
    sequence: Optional[List[PneumaticStep]] = None
    repeat: bool = False


class _StubGPIO:
    """Fallback GPIO backend for non-RPi development."""

    BCM = 11
    OUT = 0
    HIGH = 1
    LOW = 0

    def __init__(self):
        self._state: Dict[int, int] = {}

    def setmode(self, *_args, **_kwargs):
        return None

    def setwarnings(self, *_args, **_kwargs):
        return None

    def setup(self, pin: int, *_args, **_kwargs):
        self._state.setdefault(pin, self.LOW)

    def output(self, pin: int, value: int):
        self._state[pin] = int(value)

    def input(self, pin: int) -> int:
        return int(self._state.get(pin, self.LOW))

    def cleanup(self):
        self._state.clear()


class PneumaticsController:
    """Thin wrapper around RPi.GPIO (or a stub) for 4 solenoid valves."""

    def __init__(
        self,
        valve_pins: Mapping[int, int] = DEFAULT_VALVE_PINS,
        leg_to_valve: Mapping[int, int] = DEFAULT_LEG_TO_VALVE,
        active_high: bool = True,
        gpio_module=None,
    ):
        self.valve_pins = dict(valve_pins)
        self.leg_to_valve = dict(leg_to_valve)
        self.active_high = bool(active_high)
        self._gpio = gpio_module
        if self._gpio is None:
            try:
                import RPi.GPIO as GPIO  # type: ignore

                self._gpio = GPIO
            except Exception:
                self._gpio = _StubGPIO()

        self._initialized = False
        self._last_states: Dict[int, bool] = {}

    def setup(self) -> None:
        if self._initialized:
            return
        self._gpio.setmode(self._gpio.BCM)
        self._gpio.setwarnings(False)
        for _, pin in self.valve_pins.items():
            self._gpio.setup(pin, self._gpio.OUT)
        self.close_all()
        self._initialized = True

    def _to_level(self, is_open: bool) -> int:
        if self.active_high:
            return self._gpio.HIGH if is_open else self._gpio.LOW
        return self._gpio.LOW if is_open else self._gpio.HIGH

    def set_valve(self, valve_id: int, is_open: bool) -> None:
        if valve_id not in self.valve_pins:
            raise ValueError(f"Unknown valve_id {valve_id}")
        self.setup()
        pin = self.valve_pins[valve_id]
        self._gpio.output(pin, self._to_level(is_open))
        self._last_states[valve_id] = bool(is_open)

    def set_leg_valve(self, leg_index: int, is_open: bool) -> None:
        if leg_index not in self.leg_to_valve:
            raise ValueError(f"Unknown leg_index {leg_index}")
        self.set_valve(self.leg_to_valve[leg_index], is_open)

    def apply(self, states: Mapping[int, bool]) -> None:
        self.setup()
        for valve_id, is_open in states.items():
            if self._last_states.get(valve_id) != bool(is_open):
                self.set_valve(valve_id, bool(is_open))

    def close_all(self) -> None:
        for valve_id in self.valve_pins:
            self.set_valve(valve_id, False)

    def cleanup(self) -> None:
        try:
            self.close_all()
        finally:
            try:
                self._gpio.cleanup()
            except Exception:
                pass
            self._initialized = False
            self._last_states.clear()


class ActuationStateMachine:
    """Coordinates gait and pneumatics with basic safety semantics."""

    def __init__(
        self,
        control,
        pneumatics: PneumaticsController,
        tick_hz: float = 50.0,
        gait_z: int = 40,
    ):
        self.control = control
        self.pneumatics = pneumatics
        self.tick_hz = float(tick_hz)
        self.gait_z = int(gait_z)

        self._lock = threading.RLock()
        self._state: ActuationState = ActuationState.IDLE

        self._gait_cmd: Optional[GaitCommand] = None
        self._pneu_cmd: Optional[PneumaticCommand] = None

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._gait_thread: Optional[threading.Thread] = None
        self._gait_stop = threading.Event()
        self._gait_cycle_start_s: float = 0.0
        self._gait_cycle_len_s: float = 0.0

        self._sync_mode: GaitPneumaticSyncMode = GaitPneumaticSyncMode.OFF

        self._seq_index = 0
        self._seq_deadline_s = 0.0

    @property
    def state(self) -> ActuationState:
        with self._lock:
            return self._state

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._loop, name="ActuationSM", daemon=True)
            self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._stop_gait_thread()
        try:
            self.pneumatics.cleanup()
        except Exception:
            pass

    def emergency_stop(self) -> None:
        with self._lock:
            self._state = ActuationState.ESTOP
            self._gait_cmd = None
            self._pneu_cmd = None
            self._sync_mode = GaitPneumaticSyncMode.OFF
        self._stop_gait_thread()
        try:
            self.pneumatics.close_all()
        except Exception:
            pass
        try:
            self.control.relax(True)
        except Exception:
            pass

    def clear_estop(self) -> None:
        with self._lock:
            if self._state == ActuationState.ESTOP:
                self._state = ActuationState.IDLE

    def set_gait_command(self, data) -> None:
        """Set the current gait command.

        data can be a list/tuple like: ["CMD_MOVE", gait, x, y, speed, angle]
        """
        if len(data) != 6:
            raise ValueError("Gait command must have 6 fields")
        cmd = GaitCommand(tuple(str(x) for x in data))
        with self._lock:
            if self._state == ActuationState.ESTOP:
                return
            self._gait_cmd = cmd

    def clear_gait(self) -> None:
        with self._lock:
            self._gait_cmd = None

    def set_pneumatic_hold(self, valve_states: Mapping[int, bool]) -> None:
        with self._lock:
            if self._state == ActuationState.ESTOP:
                return
            self._pneu_cmd = PneumaticCommand(hold_states=dict(valve_states))

    def set_pneumatic_sequence(self, steps: List[PneumaticStep], repeat: bool = False) -> None:
        with self._lock:
            if self._state == ActuationState.ESTOP:
                return
            self._pneu_cmd = PneumaticCommand(sequence=list(steps), repeat=bool(repeat))
            self._seq_index = 0
            self._seq_deadline_s = 0.0

    def clear_pneumatics(self) -> None:
        with self._lock:
            self._pneu_cmd = None
            self._seq_index = 0
            self._seq_deadline_s = 0.0
        try:
            self.pneumatics.close_all()
        except Exception:
            pass

    def set_gait_pneumatic_sync(self, mode: GaitPneumaticSyncMode) -> None:
        with self._lock:
            self._sync_mode = GaitPneumaticSyncMode(mode)

    def _recompute_state(self) -> None:
        if self._state == ActuationState.ESTOP:
            return
        has_gait = self._gait_cmd is not None
        has_pneu = self._pneu_cmd is not None
        if has_gait and has_pneu:
            self._state = ActuationState.GAIT_AND_PNEUMATICS
        elif has_gait:
            self._state = ActuationState.GAIT_ONLY
        elif has_pneu:
            self._state = ActuationState.PNEUMATICS_ONLY
        else:
            self._state = ActuationState.IDLE

    def _ensure_gait_thread(self) -> None:
        if self._gait_thread and self._gait_thread.is_alive():
            return
        self._gait_stop.clear()
        self._gait_thread = threading.Thread(target=self._gait_loop, name="GaitRunner", daemon=True)
        self._gait_thread.start()

    def _stop_gait_thread(self) -> None:
        self._gait_stop.set()
        t = self._gait_thread
        if t and t.is_alive():
            t.join(timeout=0.5)
        self._gait_thread = None

    def _estimate_cycle_len_s(self, gait: str, speed: int) -> float:
        # Mirrors Control.run_gait() mapping from speed slider (2..10) to F.
        def map_value(value: float, from_low: float, from_high: float, to_low: float, to_high: float) -> float:
            return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

        if gait == "1":
            f = round(map_value(speed, 2, 10, 126, 22))
        else:
            f = round(map_value(speed, 2, 10, 171, 45))
        f = max(1, int(f))
        return float(f) * 0.01

    def _gait_loop(self) -> None:
        while not self._gait_stop.is_set() and not self._stop_event.is_set():
            with self._lock:
                if self._state == ActuationState.ESTOP or self._gait_cmd is None:
                    break
                data = self._gait_cmd.data
                gait = self._gait_cmd.gait
                speed = self._gait_cmd.speed
                self._gait_cycle_start_s = time.monotonic()
                self._gait_cycle_len_s = self._estimate_cycle_len_s(gait, speed)

            try:
                self.control.run_gait(list(data), Z=self.gait_z)
            except Exception:
                # If gait fails (hardware/runtime), exit gait thread and let the SM fall back to IDLE.
                with self._lock:
                    self._gait_cmd = None
                break

        with self._lock:
            self._gait_cycle_start_s = 0.0
            self._gait_cycle_len_s = 0.0

    def _current_swing_legs(self) -> Tuple[int, ...]:
        with self._lock:
            if not self._gait_cmd or self._gait_cycle_len_s <= 0:
                return ()
            now = time.monotonic()
            phase = (now - self._gait_cycle_start_s) / self._gait_cycle_len_s
            phase = phase % 1.0
            gait = self._gait_cmd.gait

        if gait == "1":
            return (0, 2) if phase < 0.5 else (1, 3)
        # gait "2": ripple order
        order = (0, 2, 1, 3)
        idx = int(phase * 4) % 4
        return (order[idx],)

    def _apply_pneumatics(self) -> None:
        with self._lock:
            pneu_cmd = self._pneu_cmd
            sync_mode = self._sync_mode
            state = self._state

        if state == ActuationState.ESTOP:
            return

        if sync_mode != GaitPneumaticSyncMode.OFF and state in (
            ActuationState.GAIT_ONLY,
            ActuationState.GAIT_AND_PNEUMATICS,
        ):
            swing = set(self._current_swing_legs())
            desired: Dict[int, bool] = {}
            for leg_idx, valve_id in self.pneumatics.leg_to_valve.items():
                is_swing = leg_idx in swing
                if sync_mode == GaitPneumaticSyncMode.SWING_OPEN:
                    desired[valve_id] = is_swing
                else:
                    desired[valve_id] = not is_swing
            self.pneumatics.apply(desired)
            return

        if pneu_cmd is None:
            if state != ActuationState.GAIT_ONLY:
                self.pneumatics.close_all()
            return

        if pneu_cmd.hold_states is not None:
            self.pneumatics.apply(pneu_cmd.hold_states)
            return

        if not pneu_cmd.sequence:
            self.pneumatics.close_all()
            return

        # Sequence mode
        now = time.monotonic()
        if self._seq_deadline_s == 0.0 or now >= self._seq_deadline_s:
            if self._seq_index >= len(pneu_cmd.sequence):
                if pneu_cmd.repeat:
                    self._seq_index = 0
                else:
                    with self._lock:
                        self._pneu_cmd = None
                    self.pneumatics.close_all()
                    return

            step = pneu_cmd.sequence[self._seq_index]
            self._seq_index += 1
            self._seq_deadline_s = now + max(0.0, float(step.duration_s))
            self.pneumatics.apply(step.valve_states)

    def _loop(self) -> None:
        period_s = 1.0 / max(1.0, self.tick_hz)
        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            with self._lock:
                self._recompute_state()
                state = self._state
                has_gait = self._gait_cmd is not None

            if state in (ActuationState.GAIT_ONLY, ActuationState.GAIT_AND_PNEUMATICS):
                if has_gait:
                    self._ensure_gait_thread()
            else:
                self._stop_gait_thread()

            try:
                self._apply_pneumatics()
            except Exception:
                # Pneumatics errors should not crash the loop.
                pass

            elapsed = time.monotonic() - loop_start
            if elapsed < period_s:
                time.sleep(period_s - elapsed)


__all__ = [
    "ActuationState",
    "GaitPneumaticSyncMode",
    "GaitCommand",
    "PneumaticStep",
    "PneumaticCommand",
    "PneumaticsController",
    "ActuationStateMachine",
]

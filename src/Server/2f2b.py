import time
import math
import threading
from typing import Dict, List

try:
    from servo import Servo
except Exception:
    # fallback stub if import fails (keeps file importable for linting)
    class Servo:
        def __init__(self): self.current_angles = [90.0]*32
        def set_servo_angle(self, ch, ang, speed=None): self.current_angles[int(ch)] = float(ang)
        def relax(self): pass

class TwoFTwoBActuator:
    """
    Simple direct-servo actuator for the 2-front / 2-back unilateral "worm" gait.
    Controls lift and extend servos per leg; hip/inner servos are intentionally unused.
    """

    def __init__(self, servo: Servo = None):
        self.servo = servo if servo is not None else Servo()
        # channels: leg -> (hip_inner, lift, extend)
        self.legs = {
            0: (15, 14, 13),  # front-right
            1: (9, 8, 31),  # back-right
            2: (22, 23, 27),  # back-left
            3: (16, 17, 18),  # front-left
        }
        self.calib_file = 'servo_calibration.txt'
        # Track all servo angles locally
        self.current_angles = [90.0] * 32

    def _get_positions(self, channels: List[int]) -> Dict[int, float]:
        return {ch: float(self.current_angles[ch]) for ch in channels}

    def _actuate_servo(self, channel: int, angle: float):
        """Actuate a servo and ensure current_angles is updated."""
        angle = max(0.0, min(180.0, angle))
        self.servo.set_servo_angle(channel, angle)
        # Store angle in local tracking
        self.current_angles[int(channel)] = angle

    def calibrate_servo_directions(self, save_path: str = 'servo_directions.txt'):
        """
        Interactively calibrate servo directions for all lift and extend motors.
        Tests each servo by moving +step and -step, asks user to confirm direction.
        Saves direction multipliers (1 or -1) to a text file.
        """
        print("\n=== Servo Direction Calibration ===")
        print("For each servo, it will move +20° and -20° from current position.")
        print("Confirm if the motion is in the intended direction.\n")
        
        direction_data = {}
        step = 20.0
        
        try:
            for leg in range(4):
                hip, lift_ch, ext_ch = self.legs[leg]
                
                # Calibrate lift motor
                print(f"\n--- Leg {leg}: Lift Motor (Channel {lift_ch}) ---")
                cur_lift = float(self.current_angles[lift_ch])
                print(f"Current angle: {cur_lift:.1f}°")
                print(f"Moving to {cur_lift + step:.1f}° (positive direction)...")
                self._actuate_servo(lift_ch, cur_lift + step)
                time.sleep(0.3)
                
                response = input("Did the leg LIFT? (y/n): ").strip().lower()
                if response == 'y':
                    direction_data[lift_ch] = 1
                    print(f"✓ Channel {lift_ch} direction: CORRECT (multiplier = 1)")
                else:
                    direction_data[lift_ch] = -1
                    print(f"✓ Channel {lift_ch} direction: INVERTED (multiplier = -1)")
                
                # Return to original
                self._actuate_servo(lift_ch, cur_lift)
                time.sleep(0.2)
                
                # Calibrate extend motor
                print(f"\n--- Leg {leg}: Extend Motor (Channel {ext_ch}) ---")
                cur_extend = float(self.current_angles[ext_ch])
                print(f"Current angle: {cur_extend:.1f}°")
                print(f"Moving to {cur_extend + step:.1f}° (positive direction)...")
                self._actuate_servo(ext_ch, cur_extend + step)
                time.sleep(0.3)
                
                response = input("Did the leg EXTEND forward? (y/n): ").strip().lower()
                if response == 'y':
                    direction_data[ext_ch] = 1
                    print(f"✓ Channel {ext_ch} direction: CORRECT (multiplier = 1)")
                else:
                    direction_data[ext_ch] = -1
                    print(f"✓ Channel {ext_ch} direction: INVERTED (multiplier = -1)")
                
                # Return to original
                self._actuate_servo(ext_ch, cur_extend)
                time.sleep(0.2)
        
        except (KeyboardInterrupt, EOFError):
            print("\nDirection calibration aborted.")
            return
        
        # Save direction data
        try:
            with open(save_path, "w") as f:
                for ch in sorted(direction_data.keys()):
                    f.write(f"{ch}\t{direction_data[ch]}\n")
            print(f"\n✓ Direction calibration saved to {save_path}")
        except Exception as e:
            print(f"Failed to save direction calibration: {e}")
            return
        
        print("Direction calibration complete!")

    def load_servo_directions(self, path: str = 'servo_directions.txt') -> Dict[int, int]:
        """Load servo direction multipliers from file."""
        data = {}
        try:
            with open(path, 'r') as f:
                for ln in f:
                    parts = ln.strip().split('\t')
                    if len(parts) >= 2:
                        try:
                            ch = int(parts[0])
                            mult = int(parts[1])
                            data[ch] = mult
                        except Exception:
                            continue
        except FileNotFoundError:
            pass  # No direction file, use default (all 1)
        except Exception as e:
            print(f'Error reading {path}: {e}')
        return data

    def unified_calibration(self, save_path: str = None):
        """
        Unified calibration function that handles both standard servo calibration
        and perpendicular motor calibration in one workflow.
        Saves all results to a single calibration file.
        """
        if save_path is None:
            save_path = self.calib_file

        calibration_data = {}

        print("\n=== Unified Servo Calibration ===")
        print("This will calibrate all lift/extend servos and perpendicular motors.")
        print("Enter angle 0-180, blank to keep current, 'q' to abort.\n")

        # Phase 1: Calibrate lift and extend servos
        print("--- Phase 1: Lift and Extend Servos ---")
        lift_extend_channels = [self.legs[l][1] for l in self.legs] + [self.legs[l][2] for l in self.legs]
        lift_extend_channels = sorted(set(lift_extend_channels))

        try:
            for ch in lift_extend_channels:
                cur = float(self.current_angles[ch])
                val = input(f"Channel {ch} (lift/extend) current {cur:.1f}° -> new angle: ").strip()
                if val == "":
                    angle = cur
                elif val.lower() == "q":
                    print("Calibration aborted.")
                    return
                else:
                    try:
                        angle = float(val)
                        angle = max(0.0, min(180.0, angle))
                    except Exception:
                        print("Invalid entry, keeping current.")
                        angle = cur
                self._actuate_servo(ch, angle)
                calibration_data[ch] = angle
                time.sleep(0.06)

            # Phase 2: Calibrate perpendicular motors
            print("\n--- Phase 2: Perpendicular Motors (9, 15, 16, 22) ---")
            perp_channels = [9, 15, 16, 22]
            for ch in perp_channels:
                cur = float(self.current_angles[ch])
                print(f"\nChannel {ch} (perpendicular): current {cur:.1f}°")
                # First move to default perpendicular (90°)
                self._actuate_servo(ch, 90.0)
                time.sleep(0.2)
                # Ask for fine-tune
                val = input(f"Enter fine-tuned angle (blank for 90.0): ").strip()
                if val == "":
                    angle = 90.0
                elif val.lower() == "q":
                    print("Calibration aborted.")
                    return
                else:
                    try:
                        angle = float(val)
                        angle = max(0.0, min(180.0, angle))
                        self._actuate_servo(ch, angle)
                    except Exception:
                        print("Invalid entry, using 90.0.")
                        angle = 90.0
                calibration_data[ch] = angle
                time.sleep(0.15)

        except (KeyboardInterrupt, EOFError):
            print("\nCalibration aborted by user.")
            return

        # Save all calibration data to single file
        try:
            with open(save_path, "w") as f:
                for ch in sorted(calibration_data.keys()):
                    f.write(f"{ch}\t{calibration_data[ch]:.1f}\n")
            print(f"\n✓ Calibration saved to {save_path}")
        except Exception as e:
            print(f"Failed to save calibration: {e}")
            return

        print("Calibration complete.")

    def load_calibration(self, path: str = None) -> Dict[int, float]:
        """Load calibration data from file."""
        if path is None:
            path = self.calib_file
        data = {}
        try:
            with open(path, 'r') as f:
                for ln in f:
                    parts = ln.strip().split('\t')
                    if len(parts) >= 2:
                        try:
                            ch = int(parts[0])
                            ang = float(parts[1])
                            data[ch] = ang
                        except Exception:
                            continue
        except FileNotFoundError:
            print(f'{path} not found')
        except Exception as e:
            print('Error reading', path, e)
        return data

    def actuate_to_calibration(self, path: str = None):
        """Move all servos present in calibration file to their calibrated angles."""
        if path is None:
            path = self.calib_file
        calib = self.load_calibration(path)
        if not calib:
            print(f'No calibration loaded from {path}')
            return
        for ch, ang in sorted(calib.items()):
            try:
                self._actuate_servo(ch, ang)
                time.sleep(0.03)
            except Exception:
                pass
        print(f'Actuated to calibration positions from {path}')

    def interactive_channel_scan(self, channels: List[int] = None, move_deg: float = 20.0):
        """
        Walk each PCA channel, move it +move_deg then back and ask user to confirm
        which physical joint moved. Saves results to 'detected_channel_map.txt'.
        """
        if channels is None:
            channels = list(range(32))
        channels = [int(c) for c in channels if 0 <= int(c) < 32]
        detected = {}
        print("Interactive channel scan. For each channel observe which servo moved and type a label (e.g. leg0_lift).")
        try:
            for ch in channels:
                cur = float(self.current_angles[ch])
                up = max(0.0, min(180.0, cur + move_deg))
                print(f"\nChannel {ch}: moving to {up:.1f} (press ENTER after observing)")
                self._actuate_servo(ch, up)
                input("Observed? (press ENTER) ")
                # move back
                self._actuate_servo(ch, cur)
                label = input(f"Label for channel {ch} (blank to skip): ").strip()
                if label:
                    detected[ch] = label
        except (KeyboardInterrupt, EOFError):
            print("\nScan interrupted.")
        # save map
        if detected:
            try:
                with open("detected_channel_map.txt", "w") as f:
                    for ch, label in sorted(detected.items()):
                        f.write(f"{ch}\t{label}\n")
                print("Saved detected_channel_map.txt")
            except Exception as e:
                print("Failed to save map:", e)
        else:
            print("No channels labeled.")

    def _clamp_servo_angle(self, ch: int, angle: float) -> float:
        """
        Clamp servo angle to ±45° of its calibrated position.
        """
        calib = self.load_calibration()
        base_angle = calib.get(ch, 90.0)
        min_angle = max(0.0, base_angle - 45.0)
        max_angle = min(180.0, base_angle + 45.0)
        return max(min_angle, min(max_angle, angle))

    def compute_ik_displacement(self, displacement_mm: float,
                               lift_link_mm: float = 90.0,
                               extend_link_mm: float = 84.0) -> tuple:
        """
        Compute IK angle deltas for a 2-DOF arm moving along the X-axis.
        
        Initial configuration:
        - Lift arm (L1=90mm): points to +X (θ1_calib = 0°)
        - Extend arm (L2=84mm): points to -Y (θ2_calib = -90°)
        - End effector at (90, -84)
        
        Forward kinematics:
        - x = L1*cos(θ1) + L2*cos(θ1 + θ2)
        - y = L1*sin(θ1) + L2*sin(θ1 + θ2)
        
        Args:
            displacement_mm: Desired displacement along +X axis
            lift_link_mm: Length of lift arm (90mm)
            extend_link_mm: Length of extend arm (84mm)
        
        Returns:
            (lift_angle_delta, extend_angle_delta) in degrees
        """
        # Calibrated angles (in radians)
        theta1_calib = 0.0  # Lift arm points to +X
        theta2_calib = -math.pi / 2.0  # Extend arm points to -Y relative to lift
        
        # Initial end effector position
        x_init = lift_link_mm * math.cos(theta1_calib) + extend_link_mm * math.cos(theta1_calib + theta2_calib)
        y_init = lift_link_mm * math.sin(theta1_calib) + extend_link_mm * math.sin(theta1_calib + theta2_calib)
        
        # Target end effector position (move along X axis)
        x_target = x_init + displacement_mm
        y_target = y_init
        
        # Compute distance from origin to target
        r_squared = x_target**2 + y_target**2
        
        # Use law of cosines to find θ2
        # r² = L1² + L2² - 2*L1*L2*cos(π - θ2) = L1² + L2² + 2*L1*L2*cos(θ2)
        cos_theta2 = (r_squared - lift_link_mm**2 - extend_link_mm**2) / (2 * lift_link_mm * extend_link_mm)
        
        # Clamp to valid range [-1, 1] to avoid numerical errors
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        
        # Take the elbow-down solution (negative sin)
        theta2 = math.acos(cos_theta2) * -1.0 if y_target < 0 else math.acos(cos_theta2)
        
        # Actually, for elbow-down (which matches our -Y configuration), use negative angle
        sin_theta2 = math.sin(theta2)
        if sin_theta2 >= 0:
            # If we got elbow-up, flip to elbow-down
            theta2 = -math.acos(cos_theta2)
        
        # Compute θ1 using atan2
        A = lift_link_mm + extend_link_mm * math.cos(theta2)
        B = extend_link_mm * math.sin(theta2)
        theta1 = math.atan2(y_target, x_target) - math.atan2(B, A)
        
        # Compute angle deltas
        delta_theta1 = theta1 - theta1_calib
        delta_theta2 = theta2 - theta2_calib
        
        # Convert from radians to degrees
        lift_angle_delta = delta_theta1 * (180.0 / math.pi)
        extend_angle_delta = delta_theta2 * (180.0 / math.pi)
        
        return (lift_angle_delta, extend_angle_delta)

    def actuate_leg_displacement(self, leg: int, displacement_mm: float, hold: float = 0.0):
        """
        Actuate lift and extend servos simultaneously using threading.
        Applies saved direction multipliers to ensure correct motion.
        """
        if leg not in self.legs:
            print(f"Invalid leg index: {leg}")
            return
        
        hip, lift_ch, ext_ch = self.legs[leg]
        
        # Read current angles fresh from servo
        cur_lift = float(self.current_angles[lift_ch])
        cur_extend = float(self.current_angles[ext_ch])
        
        lift_delta, extend_delta = self.compute_ik_displacement(displacement_mm)
        
        # Load direction multipliers
        directions = self.load_servo_directions()
        lift_mult = directions.get(lift_ch, 1)
        extend_mult = directions.get(ext_ch, 1)
        
        # Apply direction multipliers
        lift_delta *= lift_mult
        extend_delta *= extend_mult
        
        tgt_lift = cur_lift + lift_delta
        tgt_extend = cur_extend + extend_delta
        
        tgt_lift = self._clamp_servo_angle(lift_ch, tgt_lift)
        tgt_extend = self._clamp_servo_angle(ext_ch, tgt_extend)
        
        print(f"Leg {leg}: displacement={displacement_mm}mm")
        print(f"  Current: lift={cur_lift:.1f}°, extend={cur_extend:.1f}°")
        print(f"  Target: lift={tgt_lift:.1f}°, extend={tgt_extend:.1f}°")
        
        # Actuate both servos simultaneously
        lift_thread = threading.Thread(target=self._actuate_servo, args=(lift_ch, tgt_lift))
        extend_thread = threading.Thread(target=self._actuate_servo, args=(ext_ch, tgt_extend))
        
        lift_thread.start()
        extend_thread.start()
        lift_thread.join()
        extend_thread.join()
        
        if hold > 0:
            time.sleep(hold)

    def actuate_to_stationed_position(self, angle, hold: float = 0.3):
        """
        Actuate robot to stationed position by lifting each leg 15° and retracting it 15°.
        Lifts and retracts are applied simultaneously per leg using threading.
        Uses current servo angles, not calibration file.
        """
        print("\nActuating to stationed position...")
        print("Lifting and retracting each leg by 15°\n")
        
        for leg in range(4):
            hip, lift_ch, ext_ch = self.legs[leg]
            
            # Get current positions from servo
            cur_lift = float(self.current_angles[lift_ch])
            cur_extend = float(self.current_angles[ext_ch])
            
            # Load direction multipliers
            directions = self.load_servo_directions()
            lift_mult = directions.get(lift_ch, 1)
            extend_mult = directions.get(ext_ch, 1)
            
            # Apply 15° lift and 15° retract with direction multipliers
            lift_delta = angle * lift_mult
            extend_delta = -angle * extend_mult  # negative for retract
            
            tgt_lift = cur_lift + lift_delta
            tgt_extend = cur_extend + extend_delta
            
            # Clamp to safe ranges
            tgt_lift = self._clamp_servo_angle(lift_ch, tgt_lift)
            tgt_extend = self._clamp_servo_angle(ext_ch, tgt_extend)
            
            print(f"Leg {leg}:")
            print(f"  Lift:   {cur_lift:.1f}° → {tgt_lift:.1f}°")
            print(f"  Extend: {cur_extend:.1f}° → {tgt_extend:.1f}°")
            
            # Actuate lift and extend simultaneously
            lift_thread = threading.Thread(target=self._actuate_servo, args=(lift_ch, tgt_lift))
            extend_thread = threading.Thread(target=self._actuate_servo, args=(ext_ch, tgt_extend))
            
            lift_thread.start()
            extend_thread.start()
            lift_thread.join()
            extend_thread.join()
            
            time.sleep(hold)
        
        print("\n✓ Stationed position complete")

    def actuate_lift_motor(self, leg: int, lift_degrees: float):
        """
        Actuate only the lift motor of a leg by a specified amount of degrees.
        
        Args:
            leg: Leg index (0, 1, 2, or 3)
            lift_degrees: Degrees to move lift motor (positive or negative)
        """
        if leg not in self.legs:
            print(f"Invalid leg index: {leg}")
            return
        
        hip, lift_ch, ext_ch = self.legs[leg]
        
        # Read current angle fresh from servo
        cur_lift = float(self.current_angles[lift_ch])
        
        # Load direction multipliers
        directions = self.load_servo_directions()
        lift_mult = directions.get(lift_ch, 1)
        
        # Calculate target with direction multiplier
        lift_delta = lift_degrees * lift_mult
        tgt_lift = cur_lift + lift_delta
        
        # Clamp to safe range
        tgt_lift = self._clamp_servo_angle(lift_ch, tgt_lift)
        
        print(f"Leg {leg} lift: {cur_lift:.1f}° → {tgt_lift:.1f}°")
        
        try:
            self._actuate_servo(lift_ch, tgt_lift)
        except Exception as e:
            print(f"Error actuating lift motor on leg {leg}: {e}")

    def ripple_gait(self, lift_deg: float = 30.0, displacement_mm: float = 30.0, 
                    hold_lift: float = 0.2, hold_extend: float = 0.2, hold_lower: float = 0.2):
        """
        Perform a ripple gait sequence in order [0, 2, 1, 3].
        For each leg: lift -> displace forward -> lower.
        
        Args:
            lift_deg: Degrees to lift each leg (default 30°)
            displacement_mm: Forward displacement distance (default 30mm)
            hold_lift: Time to hold after lifting (seconds)
            hold_extend: Time to hold after extending (seconds)
            hold_lower: Time to hold after lowering (seconds)
        """
        order = [0, 2, 1, 3]
        
        print("\n=== Ripple Gait ===")
        print(f"Lift: {lift_deg}°, Displacement: {displacement_mm}mm\n")
        
        for leg in order:
            hip, lift_ch, ext_ch = self.legs[leg]
            
            # Read current angles fresh from servo
            orig_lift = float(self.current_angles[lift_ch])
            orig_extend = float(self.current_angles[ext_ch])
            
            # Step 1: Lift leg
            print(f"Leg {leg}: Lifting {lift_deg}°...")
            self.actuate_lift_motor(leg, lift_deg)
            time.sleep(hold_lift)
            
            # Step 2: Displace forward using IK
            print(f"Leg {leg}: Displacing {displacement_mm}mm...")
            if leg == 0 or leg == 3:
                self.actuate_leg_displacement(leg, displacement_mm, hold=hold_extend)
            elif leg == 1 or leg == 2:
                self.actuate_leg_displacement(leg, -displacement_mm, hold=hold_extend)
            
            # # Step 3: Lower leg back to original lift position
            # print(f"Leg {leg}: Lowering...")
            # self.actuate_lift_motor(leg, orig_lift - float(self.current_angles[lift_ch]))
            # time.sleep(hold_lower)
            
            # 2 second delay before next leg
            print(f"Waiting 2 seconds before next leg...\n")
            time.sleep(2.0)
        
        print("✓ Ripple gait complete")

def main():
    act = TwoFTwoBActuator()
    try:
        while True:
            print("\n2f2b actuator test menu:")
            print("1) Unified calibration (all servos + perpendicular)")
            print("2) Calibrate servo directions (lift/extend)")
            print("3) Actuate to calibration file")
            print("4) Actuate to stationed position (lift/retract variable°)")
            print("5) Interactive channel scan (identify physical servos)")
            print("6) Test leg displacement (IK-based actuation)")
            print("7) Test lift motor actuation")
            print("8) Ripple gait")
            print("9) Relax servos and exit")
            choice = input("Select [1-9]: ").strip()
            if choice == "1":
                act.unified_calibration()
            elif choice == "2":
                act.calibrate_servo_directions()
            elif choice == "3":
                act.actuate_to_calibration()
            elif choice == "4":
                act.actuate_to_stationed_position(30.0)
            elif choice == "5":
                chans = input("Channels to scan (comma list or blank for 0-31): ").strip()
                if chans:
                    try:
                        ch_list = [int(c.strip()) for c in chans.split(",") if c.strip()!='']
                    except Exception:
                        ch_list = None
                else:
                    ch_list = None
                act.interactive_channel_scan(channels=ch_list)
            elif choice == "6":
                try:
                    leg = int(input("Leg index (0-3): ").strip())
                    disp = float(input("Displacement (mm, positive = extend forward): ").strip())
                except Exception:
                    print("Invalid input")
                    continue
                print(f"Actuating leg {leg} with {disp} mm displacement...")
                act.actuate_leg_displacement(leg, disp, hold=0.5)
            elif choice == "7":
                try:
                    leg = int(input("Leg index (0-3): ").strip())
                    lift_deg = float(input("Lift degrees (positive or negative): ").strip())
                except Exception:
                    print("Invalid input")
                    continue
                act.actuate_lift_motor(leg, lift_deg)
            elif choice == "8":
                try:
                    lift_deg = float(input("Lift degrees (default 30): ").strip() or "30")
                    disp_mm = float(input("Displacement mm (default 30): ").strip() or "30")
                except Exception:
                    lift_deg = 30.0
                    disp_mm = 30.0
                act.ripple_gait(lift_deg=lift_deg, displacement_mm=disp_mm)
            elif choice == "9":
                print("Relaxing servos and exiting.")
                act.servo.relax()
                break
            else:
                print("Invalid selection.")
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted — relaxing servos.")
        try:
            act.servo.relax()
        except Exception:
            pass


if __name__ == "__main__":
    main()
# test_drone.py
import argparse
import time
import os
from dronekit import connect, APIException, VehicleMode
from ultralytics import YOLO


# INDIVIDUAL TEST FUNCTIONS


def test_battery(vehicle, min_level=30):
    """Checks if battery level is above a minimum threshold."""
    print("\n--- Testing Battery ---")
    level = vehicle.battery.level
    if level and level >= min_level:
        print(f"PASS: Battery level is {level}% (>= {min_level}%)")
        return True
    else:
        print(f"FAIL: Battery level is {level}%. Below minimum of {min_level}%.")
        return False

def test_gps(vehicle, min_sats=8):
    """Checks for a 3D GPS fix and minimum number of satellites."""
    print("\n--- Testing GPS ---")
    # Wait for the GPS information to be populated

    # gives the gps 10 seconds to get a good lock
    for _ in range(10):
        if vehicle.gps_0.fix_type >= 3:
            break
        time.sleep(1)
        
    fix_type = vehicle.gps_0.fix_type
    sats = vehicle.gps_0.satellites_visible
    
    if fix_type >= 3 and sats >= min_sats:
        print(f"PASS: GPS has 3D Fix with {sats} satellites.")
        return True
    else:
        print(f"FAIL: GPS Fix Type: {fix_type} | Satellites Visible: {sats}.")
        print("      (Note: Requires clear view of the sky)")
        return False

def test_ekf_status(vehicle):
    """Checks if the EKF (Extended Kalman Filter) is healthy."""
    print("\n--- Testing EKF Status ---")
    if vehicle.ekf_ok:
        print("PASS: EKF is healthy.")
        return True
    else:
        print("FAIL: EKF is not healthy. Check sensor calibrations and health.")
        return False

def test_vehicle_armable(vehicle):
    """Checks if the vehicle is armable, passing ArduPilot's internal checks."""
    print("\n--- Testing Vehicle Armable Status ---")
    # Wait for the vehicle to be armable
    for _ in range(5):
        if vehicle.is_armable:
            break
        time.sleep(1)

    if vehicle.is_armable:
        print("PASS: Vehicle is armable.")
        return True
    else:
        print("FAIL: Vehicle is not armable. Check GCS for specific pre-arm failure messages.")
        return False

def test_mission_files(vehicle, files_to_check):
    """Checks if all necessary mission files (KML, SHP) exist."""
    print("\n--- Testing Mission File Paths ---")
    all_found = True
    for f in files_to_check:
        if not os.path.exists(f):
            print(f"FAIL: File not found at path: {f}")
            all_found = False
    
    if all_found:
        print("PASS: All mission files found.")
        return True
    else:
        return False
    
# can omit based on the testing conditions

def test_ai_model(vehicle, model_path='yolov8n.pt'):
    """Tests if the YOLOv8 AI model can be loaded successfully."""
    print("\n--- Testing AI Model Loading ---")
    if not os.path.exists(model_path):
        print(f"FAIL: AI model file not found at path: {model_path}")
        return False
    try:
        YOLO(model_path)
        print("PASS: AI model loaded successfully.")
        return True
    except Exception as e:
        print(f"FAIL: Could not load AI model. Error: {e}")
        return False
    
# not required to run this test


##########################
# check motor speed test

def test_motor_control(vehicle):
    """
    Tests motor response by spinning them at a low throttle.
    REQUIRES USER CONFIRMATION AND PROPELLERS MUST BE OFF.
    """
    print("\n--- Testing Motor Control (PROPS OFF!) ---")
    
    # ----------------- SAFETY CONFIRMATION -----------------
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("!! DANGER: This test will spin the motors.            !!")
    print("!! REMOVE ALL PROPELLERS before continuing.           !!")
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    
    confirmation = input("--> Have you removed all propellers? (yes/no): ")
    if confirmation.lower() != 'y':
        print("FAIL: User did not confirm prop removal. Aborting motor test.")
        return False

    # ----------------- TEST LOGIC -----------------
    try:
        # Set mode to STABILIZE for direct throttle control and arm
        print("Setting mode to STABILIZE...")
        vehicle.mode = VehicleMode("STABILIZE")
        print("Arming motors...")
        vehicle.armed = True
        
        # Wait for arming to complete
        for _ in range(5):
            if vehicle.armed:
                break
            time.sleep(1)
        if not vehicle.armed:
            print("FAIL: Vehicle did not arm.")
            return False

        print("Armed. Sending low throttle command for 3 seconds...")
        
        # RC channel 3 is typically throttle.
        # PWM values are ~1000 (off) to ~2000 (full). 1100 is a safe low throttle.
        throttle_pwm = 1150
        vehicle.channels.overrides['3'] = throttle_pwm
        time.sleep(3)

        print(f"Throttle test complete. Releasing override and disarming.")

    finally:
        # ----------------- CLEANUP (CRITICAL) -----------------
        # Always clear overrides and disarm, even if there was an error
        vehicle.channels.overrides = {}
        vehicle.disarm()
        print("Disarmed and RC overrides cleared.")

    # ----------------- USER VERIFICATION -----------------
    print("\nPlease visually and audibly confirm the result.")
    final_check = input("--> Did all motors spin smoothly at a low speed? (yes/no): ")
    if final_check.lower() == 'y':
        print("PASS: confirmed motor control is working.")
        return True
    else:
        print("FAIL: reported an issue with motor spin.")
        return False


#MAIN TEST RUNNER


def run_all_tests(vehicle):
    """Runs all pre-flight checks in sequence."""
    print("\n========================")
    print("STARTING PRE-FLIGHT CHECKS")
    print("========================")

    # List of all test functions to run.
    # The second element in the tuple is a dictionary of arguments for the test.
    tests_to_run = [
        (test_battery, {'min_level': 30}),
        (test_gps, {'min_sats': 8}),
        (test_ekf_status, {}),
        (test_vehicle_armable, {}),
        (test_motor_control, {})
        # (test_mission_files, {'files_to_check': ['new_area.kml', 'Water_Polygons.shp']}),
        # (test_ai_model, {'model_path': 'yolov8n.pt'})
    ]

    for test_func, kwargs in tests_to_run:
        # Pass the vehicle object along with any other specific arguments
        if not test_func(vehicle, **kwargs):
            return False  # Stop immediately on failure
        time.sleep(1) # Small delay between tests for readability
    
    return True # All tests passed

# ####################################################################################
# ## SCRIPT EXECUTION
# ####################################################################################

def main():
    parser = argparse.ArgumentParser(description='Drone Pre-flight Test Script')
    parser.add_argument(
        '--connect',
        default='/dev/ttyAMA0',
        help="Vehicle connection string. Defaults to /dev/ttyAMA0 for RPi."
    )
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate for serial connections.")
    args = parser.parse_args()

    vehicle = None
    try:
        print(f"Connecting to vehicle on: {args.connect} with baud: {args.baud}")
        vehicle = connect(args.connect, wait_ready=True, baud=args.baud, heartbeat_timeout=60)
        print("Connection successful.")
        
        # Run all pre-flight checks
        if not run_all_tests(vehicle):
            print("\n PRE-FLIGHT CHECKS FAILED. Please resolve issues before flying.")
            return # Exit the script
            
        print("\n=======================================================")
        print("ALL PRE-FLIGHT CHECKS PASSED. Drone is mission-ready.")
        print("=======================================================")

    except APIException as e:
        print(f"Connection Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if vehicle:
            vehicle.close()
            print("\nConnection closed.")

if __name__ == '__main__':
    main()
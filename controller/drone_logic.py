# controller/drone_logic.py
import time
from dronekit import VehicleMode

def arm_and_takeoff(vehicle, target_altitude, logger):
    """
    Arms the vehicle and flies to a specified TargetAltitude.
    """
    logger.info("*****************Running basic pre-arm checks*****************")
    # Wait until the vehicle is ready to be armed
    while not vehicle.is_armable:
        logger.warning("Waiting for vehicle to initialize...")
        time.sleep(1)

    logger.info("*****************Arming motors*****************")
    # Set the vehicle's mode to GUIDED
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm the vehicle is armed before attempting to take off
    while not vehicle.armed:
        logger.warning("*****************Waiting for arming*****************")
        time.sleep(1)

    logger.info(f"Taking off to {target_altitude}m! ")
    vehicle.simple_takeoff(target_altitude)  # Take off

    # Wait until the vehicle reaches a safe height
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        # Log the current altitude for debugging
        logger.debug(f"Current Altitude: {current_altitude:.2f}m")
        
        # Break the loop when the target altitude is nearly reached
        if current_altitude >= target_altitude * 0.95:
            logger.info("Reached target altitude.")
            break
        time.sleep(1)
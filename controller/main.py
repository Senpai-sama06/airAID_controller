# controller/main.py
import argparse
import time
from dronekit import connect, APIException, VehicleMode

# Relative imports to use our other modules within the same package
from .util import setup_logger
from .drone_logic import arm_and_takeoff
from .path import *
from .path_with_flags import *
from .human_det import *

def controller():
    # --- Set up command-line argument parsing
    parser = argparse.ArgumentParser(description='AIRAID Drone Control Script')
    parser.add_argument(
        '--connect',
        required=True,
        help="Vehicle connection string. e.g., /dev/ttyAMA0 or udp:127.0.0.1:14550"
    )
    parser.add_argument(
        '--baud',
        type=int,
        default=57600,
        help="Baud rate for serial connections."
    )
    parser.add_argument(
        '--takeoff-alt',
        type=float,
        default=5.0,
        help="Target takeoff altitude in meters."
    )
    parser.add_argument(
        '--log-level',
        default='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL'],
        help='Set the logging level for the console output.'
    )
    args = parser.parse_args()

    # --- Setup the logger using the utility function
    logger = setup_logger(args.log_level)
    
    vehicle = None
    # --- Main try...finally block to ensure safe landing and disconnect
    try:
        # --- Connect to the Vehicle
        logger.info(f"Connecting to vehicle on: {args.connect}")
        vehicle = connect(args.connect, wait_ready=True, baud=args.baud)
        logger.info(">> Connection successful")

        # --- Arm and Takeoff
        arm_and_takeoff(vehicle, args.takeoff_alt, logger)
        logger.info("Takeoff complete. Ready for autonomous commands.")

        # ***************************************************************
        # This is the placeholder for your team's main mission logic.
        # This is where you'll integrate YOLO, MiDaS, and path planning.
        # ***************************************************************
        logger.info("Executing mission placeholder for 20 seconds...")
        time.sleep(20)
        logger.info("Mission placeholder finished.")

    except APIException as e:
        logger.critical(f"DroneKit API Error: {e}")
    except Exception as e:
        logger.critical(f"An unexpected error occurred: {e}", exc_info=True)
    finally:
        # This block will always run, ensuring the vehicle attempts to land safely.
        if vehicle:
            logger.warning("Mission ending. Commanding vehicle to LAND.")
            vehicle.mode = VehicleMode("LAND")
            # Wait until the vehicle is disarmed
            while vehicle.armed:
                logger.info("Waiting for disarm...")
                time.sleep(1)
            logger.info("Vehicle disarmed. Closing connection.")
            vehicle.close()
        logger.info("***************************************************")
        logger.info("*****************Script finished*******************")
        logger.info("***************************************************")

# This allows you to run the script directly for testing if needed
# (e.g., python controller/main.py --connect ...)
if __name__ == '__main__':
    controller()
import argparse
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import math

# ####################################################################################
# ## HELPER FUNCTIONS
# ####################################################################################

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobalRelative objects.

    This function is based on the Haversine formula, which calculates the great-circle
    distance between two points on a sphere given their longitudes and latitudes.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a new LocationGlobalRelative object mutated by dNorth and dEast metres.
    """
    earth_radius = 6378137.0 # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)


# ####################################################################################
# ## FLIGHT LOGIC
# ####################################################################################

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Running basic pre-arm checks...")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {aTargetAltitude}m!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

def run_flight_test(vehicle, takeoff_alt, test_dist):
    """
    Runs the full to-and-fro flight test.
    """
    # --- Takeoff
    arm_and_takeoff(vehicle, takeoff_alt)

    # --- Get Home Location
    home_point = vehicle.location.global_relative_frame
    print(f"Home location saved: Lat {home_point.lat:.6f}, Lon {home_point.lon:.6f}")

    # --- Calculate Target
    # Fly north for the specified distance
    target_point = get_location_metres(home_point, test_dist, 0)
    print(f"Calculating target point {test_dist}m North...")

    # --- Fly to Target
    print(f"Flying to target...")
    vehicle.simple_goto(target_point)

    while vehicle.mode.name=="GUIDED": #Stop action if we switch out of guided
        distance_to_target = get_distance_metres(vehicle.location.global_relative_frame, target_point)
        print(f"Distance to target: {distance_to_target:.2f}m")
        if distance_to_target < 1.0: # Arrived if within 1 meter
            print("Arrived at target point.")
            break
        time.sleep(1)

    # --- Fly back to Home
    print("Returning to launch site...")
    vehicle.simple_goto(home_point)

    while vehicle.mode.name=="GUIDED":
        distance_to_home = get_distance_metres(vehicle.location.global_relative_frame, home_point)
        print(f"Distance to home: {distance_to_home:.2f}m")
        if distance_to_home < 1.0:
            print("Arrived back at home.")
            break
        time.sleep(1)

    # --- Land
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    # Wait until the vehicle is disarmed
    while vehicle.armed:
        print(" Waiting for disarm...")
        time.sleep(1)
    
    print("Flight test complete. Vehicle is disarmed.")


# ####################################################################################
# ## SCRIPT EXECUTION
# ####################################################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs an automated flight test.')
    parser.add_argument('--connect', default='/dev/ttyAMA0', help="Vehicle connection string.")
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--altitude', type=int, default=10, help="Target takeoff altitude in meters.")
    parser.add_argument('--distance', type=int, default=10, help="Distance in meters for the 'to-and-fro' flight leg.")
    args = parser.parse_args()

    vehicle = None
    try:
        print(f"Connecting to vehicle on: {args.connect}...")
        vehicle = connect(args.connect, wait_ready=True, baud=args.baud)
        
        # Run the flight test
        run_flight_test(vehicle, args.altitude, args.distance)

    except APIException as e:
        print(f"DroneKit API Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if vehicle:
            vehicle.close()
            print("Connection closed.")
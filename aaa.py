from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import keyboard

# --- CONNECT TO SIM ---
print("Connecting to CoppeliaSim...")
client = RemoteAPIClient()
sim = client.require('sim')

sim.startSimulation()
print("Simulation started.")

# --- GET ROBOT AND SENSOR HANDLES ---
robot = sim.getObject("/PioneerP3DX")
left_motor = sim.getObject("/PioneerP3DX/leftMotor")
right_motor = sim.getObject("/PioneerP3DX/rightMotor")
ultrasound_sensor = sim.getObject("/PioneerP3DX/ultrasonicSensor[0]")

# --- PARAMETERS ---
distance_threshold = 0.2  # meters
mapping_data = []

# --- HELPER FUNCTIONS ---
def read_ultrasound():
    detection_state, detected_point, _, _, _ = sim.readProximitySensor(ultrasound_sensor)
    if not detection_state:
        return 1.0
    if isinstance(detected_point, (list, tuple)) and len(detected_point) >= 3:
        return (detected_point[0]**2 + detected_point[1]**2 + detected_point[2]**2) ** 0.5
    elif isinstance(detected_point, (float, int)):
        return float(detected_point)
    return 1.0

def move_robot(forward_speed=0, turn_speed=0):
    sim.setJointTargetVelocity(left_motor, forward_speed - turn_speed)
    sim.setJointTargetVelocity(right_motor, forward_speed + turn_speed)

# --- MAIN LOOP ---
try:
    print("Control the robot with W A S D. Press Ctrl+C to stop.")
    while True:
        dist = read_ultrasound()
        mapping_data.append(dist)

        # Proximity safety stop
        if dist < distance_threshold:
            move_robot(0, 0)
        else:
            if keyboard.is_pressed('w'):
                move_robot(10, 10)
            elif keyboard.is_pressed('s'):
                move_robot(-10, -10)
            elif keyboard.is_pressed('a'):
                move_robot(5, 10)
            elif keyboard.is_pressed('d'):
                move_robot(5, -10)
            else:
                move_robot(0, 0)

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopping robot and ending simulation...")
    move_robot(0, 0)
    sim.stopSimulation()
    print("Simulation stopped.")

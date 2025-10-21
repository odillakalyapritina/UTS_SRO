#!/usr/bin/env python3
# -- coding: utf-8 --
"""
Pioneer P3DX Mapping (Keyboard Control + Ultrasonic Sensor[3])
Transforms sensor readings -> body -> world (rotation about Z + translation)
Saves mapping points and robot path, writes SVG and text file on exit.
"""
import math
import time
from datetime import datetime
import sys
import numpy as np
import keyboard
import matplotlib.pyplot as plt

# remote API client
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# -------------------------
# Helper / configuration
# -------------------------
DEFAULT_SENSOR_OFFSET_BODY = np.array([0.10, 0.0, 0.15])  # [forward, right, up] (meters)
SENSOR_INDEX = 3  # use ultrasonicSensor[3]
SLEEP_STEP = 0.06  # seconds per loop
DISTANCE_THRESHOLD = 0.20  # meters for proximity stop

# initial speeds
BASE_SPEED = 4.0
TURN_SPEED = 3.0

# -------------------------
# Connect to CoppeliaSim
# -------------------------
print("🔄 Connecting to CoppeliaSim (ZMQ remote API)...")
try:
    client = RemoteAPIClient()
    sim = client.require('sim')
except Exception as e:
    print("❌ Failed to connect to CoppeliaSim (is the ZMQ plugin active and CoppeliaSim running?).")
    print("Error:", e)
    sys.exit(1)

# ensure simulation state
if sim.getSimulationState() != sim.simulation_stopped:
    try:
        sim.stopSimulation()
        time.sleep(0.3)
    except:
        pass

sim.setStepping(False)
sim.startSimulation()
time.sleep(0.2)
print("✅ Simulation started.")

# -------------------------
# Safe handle lookup
# -------------------------
def get_handle_try_variants(*variants):
    """Try to get object handle for a list of possible names; return first that exists."""
    for name in variants:
        try:
            return sim.getObjectHandle(name)
        except Exception:
            # try also the path-style getObject (some scenes use strict paths)
            try:
                return sim.getObject(name)
            except Exception:
                continue
    return None

# common name variants for Pioneer P3DX objects (scene-dependent)
robot = get_handle_try_variants('Pioneer_p3dx', 'PioneerP3DX', '/PioneerP3DX', 'Pioneer_pioneer_p3dx')
left_motor = get_handle_try_variants('Pioneer_p3dx_leftMotor', '/PioneerP3DX/leftMotor', 'leftMotor')
right_motor = get_handle_try_variants('Pioneer_p3dx_rightMotor', '/PioneerP3DX/rightMotor', 'rightMotor')
# ultrasonic sensor #3 common names
ultra3 = get_handle_try_variants('Pioneer_p3dx_ultrasonicSensor3',
                                 '/PioneerP3DX/ultrasonicSensor[3]',
                                 'ultrasonicSensor3',
                                 '/PioneerP3DX/ultrasonicSensor')

if not (robot and left_motor and right_motor and ultra3):
    print("❌ Could not find required objects in the scene. Handles:")
    print(" robot:", robot, " left_motor:", left_motor, " right_motor:", right_motor, " ultra3:", ultra3)
    print("Make sure scene contains Pioneer P3DX and its ultrasonic sensor #3.")
    try:
        sim.stopSimulation()
    except:
        pass
    sys.exit(1)

print("✅ All needed handles found.")

# -------------------------
# Transform functions
# -------------------------
def read_proximity(sensor_handle):
    """
    Read proximity sensor and return detection flag and detected_point (3-vector in sensor frame)
    sim.readProximitySensor returns: (result, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector)
    """
    try:
        res = sim.readProximitySensor(sensor_handle)
    except Exception as e:
        # Some remote API wrappers return tuple directly; handle both ways
        print("⚠ readProximitySensor error:", e)
        return False, None
    # normalize returned shape
    if isinstance(res, (list, tuple)):
        # many wrappers: result, detected, point, *_ = res
        if len(res) >= 3:
            result = res[0]
            detected = res[1]
            point = res[2]
            if isinstance(point, (list, tuple, np.ndarray)) and len(point) >= 3:
                return bool(detected), np.array(point[:3], dtype=float)
            else:
                return bool(detected), None
    return False, None

def get_robot_pose(robot_handle):
    """Return (pos3, ori3) where pos is world position (x,y,z) and ori is Euler angles (alpha,beta,yaw)."""
    pos = sim.getObjectPosition(robot_handle, -1)  # array-like
    ori = sim.getObjectOrientation(robot_handle, -1)
    return np.array(pos, dtype=float), np.array(ori, dtype=float)

def sensor_point_to_body(sensor_point, sensor_offset=DEFAULT_SENSOR_OFFSET_BODY, sensor_angle=0.0):
    """
    Convert a detected point (in sensor frame) to robot body frame.
    sensor_point: np.array([x,y,z]) reported by readProximitySensor (usually in sensor local coords)
    sensor_offset: [forward, right, up] offset of sensor origin relative to robot body origin (meters)
    sensor_angle: rotation of sensor relative to robot body in the XY plane (radians) (optional)
    """
    # rotate sensor_point by sensor_angle in XY plane if needed, then translate by sensor_offset
    x_s, y_s, z_s = sensor_point
    # rotate in sensor XY plane
    ca = math.cos(sensor_angle)
    sa = math.sin(sensor_angle)
    x_rot = x_s * ca - y_s * sa
    y_rot = x_s * sa + y_s * ca
    z_rot = z_s
    # sensor_offset convention: [forward, right, up]
    # body coordinates assume x forward, y right, z up
    p_body = np.array([x_rot + sensor_offset[0], y_rot + sensor_offset[1], z_rot + sensor_offset[2]])
    return p_body

def body_to_world(p_body, robot_pos, robot_ori):
    """
    Transform a 3D point in robot body frame to world frame using yaw rotation + translation.
    robot_pos: world position [x,y,z]
    robot_ori: Euler angles [alpha, beta, yaw]
    Returns np.array([xw, yw, zw])
    """
    yaw = robot_ori[2]
    ca = math.cos(yaw)
    sa = math.sin(yaw)
    Rz = np.array([[ca, -sa, 0],
                   [sa,  ca, 0],
                   [ 0,   0, 1]])
    p_world = np.array(robot_pos) + Rz.dot(p_body)
    return p_world

# -------------------------
# Motion control
# -------------------------
class DriveController:
    def __init__(self, left_h, right_h, base_speed=BASE_SPEED, turn_speed=TURN_SPEED):
        self.left = left_h
        self.right = right_h
        self.base_speed = base_speed
        self.turn_speed = turn_speed

    def set_velocity(self, left_vel, right_vel):
        sim.setJointTargetVelocity(self.left, float(left_vel))
        sim.setJointTargetVelocity(self.right, float(right_vel))

    def move(self, linear, angular):
        vL = vR = 0.0
        if linear > 0:
            vL = vR = self.base_speed * linear
        elif linear < 0:
            vL = vR = -self.base_speed * (-linear)
        if angular != 0:
            turn = self.turn_speed * angular
            vL -= turn
            vR += turn
        self.set_velocity(vL, vR)
        return vL, vR

    def stop(self):
        self.set_velocity(0, 0)


class P3DXMapper:
    def __init__(self, robot_h, left_h, right_h, sensor_h):
        self.robot = robot_h
        self.left = left_h
        self.right = right_h
        self.sensor = sensor_h
        self.controller = DriveController(left_h, right_h)
        self.map_points = []   # list of (x, y, z)
        self.robot_path = []   # list of (x, y)
        self.sensor_angle = 0.0
        self.sensor_offset = DEFAULT_SENSOR_OFFSET_BODY

    def read_and_map(self):
        detected, point = read_proximity(self.sensor)
        pos, ori = get_robot_pose(self.robot)
        # store robot path (x,y)
        self.robot_path.append((pos[0], pos[1]))
        if detected and point is not None:
            # point is given in sensor coordinates (relative to sensor)
            p_body = sensor_point_to_body(point, sensor_offset=self.sensor_offset, sensor_angle=self.sensor_angle)
            p_world = body_to_world(p_body, pos, ori)
            self.map_points.append((p_world[0], p_world[1], p_world[2]))
            print(f"📍 detected -> world: x={p_world[0]:.3f}, y={p_world[1]:.3f}, z={p_world[2]:.3f}")
            return p_world
        else:
            # no detection
            return None

    def stop(self):
        self.controller.stop()

    def save_data(self):
        # text file with x,y,z per line
        txt_name = datetime.now().strftime("p3dx_mapping_%Y%m%d_%H%M%S.txt")
        with open(txt_name, "w") as f:
            for p in self.map_points:
                f.write(f"{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}\n")
        print(f"💾 Mapping points saved to {txt_name}")

        # robot path
        path_name = datetime.now().strftime("p3dx_path_%Y%m%d_%H%M%S.txt")
        with open(path_name, "w") as f:
            for px, py in self.robot_path:
                f.write(f"{px:.6f},{py:.6f}\n")
        print(f"💾 Robot path saved to {path_name}")

    def plot_svg(self):
        if not self.map_points:
            print("⚠ No mapping points to plot.")
            return
        pts = np.array(self.map_points)
        fig = plt.figure(figsize=(7,7))
        ax = fig.add_subplot(111)
        ax.scatter(pts[:,0], pts[:,1], s=12, label='Obstacles', zorder=2)
        if self.robot_path:
            rp = np.array(self.robot_path)
            ax.plot(rp[:,0], rp[:,1], linewidth=1, label='Robot Path', zorder=1)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("P3DX Mapping (Ultrasonic Sensor[3]) - World Coordinates")
        ax.axis('equal')
        ax.grid(True)
        ax.legend()
        svg_name = datetime.now().strftime("p3dx_map_%Y%m%d_%H%M%S.svg")
        plt.savefig(svg_name, format='svg', bbox_inches='tight', dpi=300)
        plt.close(fig)
        print(f"🗺 Saved SVG map as {svg_name}")

# -------------------------
# Main loop
# -------------------------
def main():
    mapper = P3DXMapper(robot, left_motor, right_motor, ultra3)
    ctrl = mapper.controller
    print("""
---------------------------------------------
🤖 P3DX Mapping Mode (Keyboard)
W : forward
S : backward
A : turn left
D : turn right
SPACE : stop
+ / - : increase / decrease speeds
Q : quit and save
---------------------------------------------
""")
    try:
        last_speed_change = 0
        while True:
            # read and map sensor (non-blocking)
            mapped = mapper.read_and_map()

            # if detected and close, stop
            if mapped is not None:
                # compute distance in sensor frame (we used p_world, but we can compute distance by projecting to robot origin)
                pos, ori = get_robot_pose(robot)
                dx = mapped[0] - pos[0]
                dy = mapped[1] - pos[1]
                dz = mapped[2] - pos[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist < DISTANCE_THRESHOLD:
                    ctrl.stop()
                    print("⚠ Obstacle too close — stopping motors.")
            # keyboard control
            linear = 0.0
            angular = 0.0
            if keyboard.is_pressed('w'):
                linear = 1.0
            elif keyboard.is_pressed('s'):
                linear = -1.0
            if keyboard.is_pressed('a'):
                angular = 0.8
            elif keyboard.is_pressed('d'):
                angular = -0.8

            # speed adjustments
            if (keyboard.is_pressed('+') or keyboard.is_pressed('=')) and (time.time() - last_speed_change) > 0.25:
                mapper.controller.base_speed = min(mapper.controller.base_speed + 0.5, 12.0)
                mapper.controller.turn_speed = min(mapper.controller.turn_speed + 0.5, 8.0)
                last_speed_change = time.time()
                print(f"⚙ Speeds -> base: {mapper.controller.base_speed:.1f}, turn: {mapper.controller.turn_speed:.1f}")
            if (keyboard.is_pressed('-') or keyboard.is_pressed('_')) and (time.time() - last_speed_change) > 0.25:
                mapper.controller.base_speed = max(mapper.controller.base_speed - 0.5, 0.5)
                mapper.controller.turn_speed = max(mapper.controller.turn_speed - 0.5, 0.5)
                last_speed_change = time.time()
                print(f"⚙ Speeds -> base: {mapper.controller.base_speed:.1f}, turn: {mapper.controller.turn_speed:.1f}")

            if keyboard.is_pressed(' '):
                ctrl.stop()

            if keyboard.is_pressed('q'):
                print("\n💾 Exiting and saving mapping data...")
                break

            ctrl.move(linear, angular)

            # step simulation if stepping is enabled (we used continuous sim by default)
            try:
                sim.step()
            except Exception:
                pass

            time.sleep(SLEEP_STEP)

    except KeyboardInterrupt:
        print("\n⏹ Interrupted by user.")
    finally:
        # stop motors & simulation, save and plot
        try:
            mapper.stop()
            sim.stopSimulation()
            time.sleep(0.2)
        except Exception:
            pass

        mapper.save_data()
        mapper.plot_svg()
        print("✅ Done. Exiting.")

if __name__ == "__main__":
    main()

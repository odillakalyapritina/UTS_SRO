import math
import time
import keyboard
import matplotlib.pyplot as plt
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# --- Koneksi ke CoppeliaSim ---
client = RemoteAPIClient()
sim = client.getObject('sim')

# --- Ambil handle objek (ubah sesuai nama di scene anda) ---
leftMotor = sim.getObject('/PioneerP3DX/leftMotor')
rightMotor = sim.getObject('/PioneerP3DX/rightMotor')
sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[3]')
robot = sim.getObject('/PioneerP3DX')

# --- Inisialisasi data ---
mapping_points = []
robot_path = []

print("🚗 Mapping dimulai... Tekan ESC untuk berhenti dan simpan hasil.")

try:
    while not keyboard.is_pressed('esc'):
        # === ambil posisi robot ===
        robot_pos = sim.getObjectPosition(robot, -1)
        robot_ori = sim.getObjectOrientation(robot, -1)
        theta_r = robot_ori[2]
        robot_path.append((robot_pos[0], robot_pos[1]))

        # === baca sensor ===
        result, distance, detectedPoint, detectedObj, _ = sim.readProximitySensor(sensor)

        if result:
            # posisi sensor relatif terhadap body (offset di depan robot)
            x_s, y_s, theta_s = 0.2, 0.0, 0.0

            # posisi objek relatif terhadap body
            x_b = x_s + distance * math.cos(theta_s)
            y_b = y_s + distance * math.sin(theta_s)

            # transformasi ke koordinat dunia
            x_w = robot_pos[0] + math.cos(theta_r) * x_b - math.sin(theta_r) * y_b
            y_w = robot_pos[1] + math.sin(theta_r) * x_b + math.cos(theta_r) * y_b

            mapping_points.append((x_w, y_w))

        time.sleep(0.05)

except KeyboardInterrupt:
    pass

# === Simpan hasil mapping setelah ESC ditekan ===
if len(mapping_points) > 0:
    plt.figure(figsize=(8, 6))
    plt.title("Mapping Pioneer P3DX")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.axis("equal")
    plt.grid(True)
    plt.plot([p[0] for p in robot_path], [p[1] for p in robot_path], 'b-', label="Lintasan Robot")
    plt.plot([p[0] for p in mapping_points], [p[1] for p in mapping_points], 'r.', markersize=3, label="Deteksi Sensor")
    plt.legend()
    plt.savefig("mapping_result.svg", format='svg')
    plt.show()
    print("✅ File SVG berhasil disimpan: mapping_result.svg")
else:
    print("⚠️ Tidak ada hasil mapping — pastikan sensor mendeteksi objek sebelum tekan ESC.")

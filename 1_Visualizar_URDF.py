import serial
import time
import numpy as np
import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Configura el puerto serial (ajústalo a tu caso)
ser = serial.Serial('COM7', 115200, timeout=1)

# --- Funciones auxiliares ---

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convierte Euler ZYX (yaw-pitch-roll) a matriz de rotación"""
    r = np.deg2rad(roll)
    p = np.deg2rad(pitch)
    y = np.deg2rad(yaw)

    R_x = np.array([[1, 0, 0],
                    [0, np.cos(r), -np.sin(r)],
                    [0, np.sin(r), np.cos(r)]])
    R_y = np.array([[np.cos(p), 0, np.sin(p)],
                    [0, 1, 0],
                    [-np.sin(p), 0, np.cos(p)]])
    R_z = np.array([[np.cos(y), -np.sin(y), 0],
                    [np.sin(y), np.cos(y), 0],
                    [0, 0, 1]])

    return R_z @ R_y @ R_x


def parse_imu_line(line):
    """Extrae aceleración y ángulos de la línea enviada por Arduino"""
    pattern = r"Acel\[g\]: ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+) \| Gyro\[°/s\]: ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+) \| Ang\[°\]: ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+)"
    match = re.match(pattern, line.strip())
    if match:
        vals = [float(x) for x in match.groups()]
        accel = np.array(vals[0:3])
        gyro  = np.array(vals[3:6])
        angle = np.array(vals[6:9])
        return accel, gyro, angle
    return None, None, None


# --- Configura la figura 3D ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_xlabel('X (global)')
ax.set_ylabel('Y (global)')
ax.set_zlabel('Z (global)')
ax.set_title('Orientación IMU WT61C')

# Define los vértices de un cubo unitario
cube_vertices = np.array([
    [-0.5, -0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [ 0.5, -0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [-0.5,  0.5,  0.5]
])

# Caras del cubo (por índices)
cube_faces = [
    [0,1,2,3],
    [4,5,6,7],
    [0,1,5,4],
    [2,3,7,6],
    [1,2,6,5],
    [0,3,7,4]
]


def draw_cube(ax, R):
    """Dibuja el cubo rotado según la matriz R"""
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X (global)')
    ax.set_ylabel('Y (global)')
    ax.set_zlabel('Z (global)')
    ax.set_title('Orientación IMU WT61C')

    rotated = (R @ cube_vertices.T).T

    # Dibuja las caras
    for face in cube_faces:
        f = [rotated[i] for i in face]
        f = np.array(f)
        ax.plot_trisurf(f[:,0], f[:,1], f[:,2], color='skyblue', alpha=0.6, edgecolor='gray')

    # Dibuja ejes del sensor
    origin = np.array([[0,0,0]])
    axis_len = 0.7
    ax.quiver(0,0,0, R[0,0]*axis_len, R[1,0]*axis_len, R[2,0]*axis_len, color='r', label='X_s')
    ax.quiver(0,0,0, R[0,1]*axis_len, R[1,1]*axis_len, R[2,1]*axis_len, color='g', label='Y_s')
    ax.quiver(0,0,0, R[0,2]*axis_len, R[1,2]*axis_len, R[2,2]*axis_len, color='b', label='Z_s')
    ax.legend()


# --- Loop principal ---
print("Leyendo datos del puerto serial y mostrando orientación 3D...")
plt.ion()

while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        if not line:
            continue

        accel, gyro, angle = parse_imu_line(line)
        if accel is None:
            continue

        roll, pitch, yaw = angle
        R = euler_to_rotation_matrix(roll, pitch, yaw)

        # Transformación de aceleraciones
        accel_global = R @ accel

        # Muestra info en consola
        print(f"Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}° | Accel Global[g]={accel_global}")

        # Actualiza el cubo 3D
        draw_cube(ax, R)
        plt.pause(0.001)

    except KeyboardInterrupt:
        print("\nFinalizando...")
        ser.close()
        break
    
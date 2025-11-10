import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph import opengl as gl
from PyQt6.QtWidgets import QApplication
from PyQt6 import QtGui
import trimesh
import re
import sys

# --- Configurar puerto serial ---
ser = serial.Serial('COM7', 115200, timeout=1)

# --- Función para convertir ángulos a matriz de rotación ---
def euler_to_rotation_matrix(roll, pitch, yaw):
    r, p, y = np.deg2rad([roll, pitch, yaw])
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

# --- Expresión regular para extraer los ángulos ---
pattern = r"Ang\[°\]: ([\-\d\.]+), ([\-\d\.]+), ([\-\d\.]+)"

# --- Inicializar aplicación Qt ---
app = QApplication(sys.argv)
view = gl.GLViewWidget()
view.setWindowTitle('Modelo 3D controlado por IMU WT61C')
view.setCameraPosition(distance=3, elevation=25, azimuth=90)
view.show()

# --- Agregar ejes y cuadrícula ---
grid = gl.GLGridItem()
grid.scale(1, 1, 1)
view.addItem(grid)

axis = gl.GLAxisItem()
axis.setSize(1, 1, 1)
view.addItem(axis)

# --- Cargar modelo OBJ con trimesh ---
mesh_trimesh = trimesh.load('humano.obj')

# 🔹 Calcular el centro y tamaño del modelo
center = mesh_trimesh.centroid
size = mesh_trimesh.scale

vertices = np.array(mesh_trimesh.vertices) - center  # Centrar en el origen
faces = np.array(mesh_trimesh.faces)

# Crear el MeshData desde los vértices/caras cargados
mesh_data = gl.MeshData(vertexes=vertices, faces=faces)

# Crear el objeto 3D (modelo humano)
mesh = gl.GLMeshItem(
    meshdata=mesh_data,
    smooth=True,
    color=(0.7, 0.7, 0.9, 1),
    shader='shaded',
    drawEdges=False
)

# 🔹 Escalar y ponerlo de pie
mesh.scale(0.07, 0.07, 0.07)
mesh.rotate(-90, 1, 0, 0, local=False)   # De acostado a vertical
mesh.rotate(180, 0, 1, 0, local=False)   # Gira para que mire hacia adelante
mesh.translate(0, 0, 0.75)              # Baja un poco para que toque el suelo
view.addItem(mesh)

# --- Crear flecha (eje del sensor) ---
arrow_vertices = np.array([[0, 0, 0], [0, 0, 1]])  # Vector Z
arrow = gl.GLLinePlotItem(pos=arrow_vertices, color=(1, 0, 0, 1), width=3, antialias=True)
view.addItem(arrow)

print("✅ Iniciando visualización 3D en tiempo real...")

# --- Bucle principal ---
while True:
    try:
        line = ser.readline().decode(errors='ignore').strip()
        match = re.search(pattern, line)
        if not match:
            continue

        roll, yaw, pitch = [float(x) for x in match.groups()]
        R = euler_to_rotation_matrix(roll, pitch, yaw)

        # Crear matriz de transformación homogénea
        M = np.eye(4)
        M[:3, :3] = R

        # Convertir numpy -> QMatrix4x4
        qmat = QtGui.QMatrix4x4(*M.T.flatten())

        # Aplicar rotación al modelo
        mesh.resetTransform()
        mesh.scale(0.07, 0.07, 0.07)
        mesh.rotate(-90, 1, 0, 0, local=False)
        mesh.rotate(180, 0, 1, 0, local=False)
        mesh.translate(0, 0, 0.75)
        mesh.applyTransform(qmat, local=True)

        # Actualizar flecha (eje Z del sensor)
        arrow_dir = (R @ np.array([0, 0, 1])) * 1.0
        arrow_vertices = np.array([[0, 0, 0], arrow_dir])
        arrow.setData(pos=arrow_vertices)

        # Actualizar ventana
        pg.QtWidgets.QApplication.processEvents()

    except KeyboardInterrupt:
        print("\nFinalizando...")
        ser.close()
        break


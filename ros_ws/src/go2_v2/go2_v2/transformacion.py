"""Matematica de los sensores del Go2 (sin ROS: se prueba sola, ver v2/pruebas/prueba_transformacion.py).

Es EXACTAMENTE la de transform_everything.py del stack de CMU (autonomy_stack_go2), que es la que
Point-LIO espera, pero vectorizada con numpy. La original recorria la nube punto por punto en
Python (read_points_list + un for + del de a uno): en una laptop eso agregaba la "demora de mas de
1 s" que CMU reporta fuera del perro. Aca una nube del L1 (~5-10 mil puntos) toma ~1 ms.

Marcos:
  L1 (utlidar_lidar): el lidar va boca abajo, inclinado ~15 grados hacia el suelo.
  body: el marco que usa Point-LIO = el L1 "enderezado" (z arriba), 4.7 cm mas abajo.
"""
import math

import numpy as np

PITCH_L1 = 2.87820258505555555556         # rad: giro del L1 respecto del cuerpo (URDF de Unitree)
CAM_OFFSET = 0.046825                       # m: altura del L1 sobre 'body' (se resta a z)
THETA_IMU = 15.1 / 180.0 * 3.1415926        # rad: inclinacion de la IMU del L1 (como en CMU)

# caja del propio perro en el marco 'body' (patas y lomo que el L1 ve): se borra
CAJA_X = (-0.7, -0.1)
CAJA_Y = (-0.3, 0.3)
CAJA_Z = (-0.6 - CAM_OFFSET, 0.0 - CAM_OFFSET)

CALIB_DEFECTO = {
    "acc_bias_x": 0.0, "acc_bias_y": 0.0, "acc_bias_z": 0.0,
    "ang_bias_x": 0.0, "ang_bias_y": 0.0, "ang_bias_z": 0.0,
    "ang_z2x_proj": 0.15, "ang_z2y_proj": -0.28,
}


def quat_desde_euler(roll, pitch, yaw):
    """Igual que tf_transformations.quaternion_from_euler(roll, pitch, yaw) (ejes 'sxyz').
    Devuelve (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def quat_por(a, b):
    """Producto de cuaterniones (x, y, z, w), igual que tf_transformations.quaternion_multiply."""
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def matriz_de_quat(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


Q_NUBE = quat_desde_euler(0.0, PITCH_L1, 0.0)            # body <- L1 (nube)
Q_IMU = quat_desde_euler(0.0, PITCH_L1, 3.14159265358)   # body <- IMU del L1
R_NUBE = matriz_de_quat(Q_NUBE)


def transformar_xyz(xyz, caja_x_max=CAJA_X[1]):
    """(n, 3) en el marco del L1 -> (n, 3) en 'body' y mascara de los puntos que NO son del perro.
    caja_x_max: hasta donde (adelante del L1) llega la caja del perro. CMU: -0.1, con la zona ciega de
    Point-LIO en 0.5 m las patas de adelante quedaban tapadas igual; con la zona ciega en 0.3 m hay que
    llevarla a +0.05 (las patas de adelante pisan hasta ~0.0 del L1)."""
    p = np.asarray(xyz, np.float64) @ R_NUBE.T
    p[:, 2] -= CAM_OFFSET
    en_caja = ((p[:, 0] > CAJA_X[0]) & (p[:, 0] < caja_x_max)
               & (p[:, 1] > CAJA_Y[0]) & (p[:, 1] < CAJA_Y[1])
               & (p[:, 2] > CAJA_Z[0]) & (p[:, 2] < CAJA_Z[1]))
    return p, ~en_caja


def transformar_imu(giro, acel, calib):
    """giro, acel: (x, y, z) de /utlidar/imu. Devuelve (giro, acel) en 'body', ya corregidos con la
    calibracion ('go2v2 calibrar'). Misma cuenta que CMU (incluye la proyeccion z->x, z->y)."""
    th = THETA_IMU
    c, s = math.cos(th), math.sin(th)
    x, y, z = giro[0], -giro[1], -giro[2]
    x2 = c * x - s * z
    y2 = y
    z2 = s * x + c * z
    x2 -= calib["ang_bias_x"]
    y2 -= calib["ang_bias_y"]
    z2 -= calib["ang_bias_z"]
    x2 += calib["ang_z2x_proj"] * z2
    y2 += calib["ang_z2y_proj"] * z2
    ax, ay, az = acel[0], -acel[1], -acel[2]
    ax2 = c * ax - s * az
    ay2 = ay
    az2 = s * ax + c * az
    return ((x2, y2, z2),
            (ax2 - calib["acc_bias_x"], ay2 - calib["acc_bias_y"], az2 - calib["acc_bias_z"]))


def orientacion_imu(q):
    """Orientacion de la IMU (x, y, z, w) llevada a 'body' (Q_IMU * q, como CMU)."""
    return quat_por(Q_IMU, q)


def leer_calibracion(ruta):
    """El yaml que escribe calibrate_imu (~/Desktop/imu_calib_data.yaml). Si falta: valores de CMU."""
    d = dict(CALIB_DEFECTO)
    try:
        import yaml
        with open(ruta) as f:
            leido = yaml.safe_load(f) or {}
        for k in d:
            if k in leido:
                d[k] = float(leido[k])
        return d, True
    except (OSError, ValueError, TypeError, ImportError):
        return d, False
    except Exception:          # yaml.YAMLError y similares: archivo roto = valores de CMU
        return d, False


def transformar_nube_bytes(datos, n, paso, offsets, caja_x_max=CAJA_X[1]):
    """La nube cruda (bytes de un PointCloud2, n puntos de 'paso' bytes; x, y, z float32 en
    'offsets') -> (bytes de la nube en 'body' SIN los puntos del perro, puntos que quedan).
    Los demas campos (intensity, ring, time) van tal cual: Point-LIO los usa."""
    crudo = np.frombuffer(datos, dtype=np.uint8, count=n * paso).reshape(n, paso).copy()
    xyz = np.empty((n, 3), np.float64)
    for i, o in enumerate(offsets):
        xyz[:, i] = np.ascontiguousarray(crudo[:, o:o + 4]).view(np.float32)[:, 0]
    p, ok = transformar_xyz(xyz, caja_x_max)
    p32 = np.ascontiguousarray(p.astype(np.float32))
    for i, o in enumerate(offsets):
        crudo[:, o:o + 4] = np.ascontiguousarray(p32[:, i]).view(np.uint8).reshape(n, 4)
    crudo = crudo[ok]
    return crudo.tobytes(), int(ok.sum())

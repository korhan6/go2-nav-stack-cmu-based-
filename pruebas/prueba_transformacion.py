"""La transformacion vectorizada (go2_v2/transformacion.py) da EXACTAMENTE lo mismo que el
transform_everything.py original de CMU (punto por punto). Sin ROS: python3 v2/pruebas/prueba_transformacion.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2 import transformacion as T  # noqa: E402

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    Rotation = None


def original_nube(points):
    """Copia de la cuenta de CMU (transform_everything.cloud_callback) con su for punto por punto."""
    q = T.quat_desde_euler(0, 2.87820258505555555556, 0)
    if Rotation is not None:
        mat = Rotation.from_quat(q).as_matrix()            # independiente de matriz_de_quat
    else:
        mat = T.matriz_de_quat(q)
    pts = np.array(points, dtype=np.float64)
    pts[:, 0:3] = pts[:, 0:3] @ mat.T
    pts[:, 2] -= 0.046825
    lista = pts.tolist()
    quitar = []
    for i, p in enumerate(lista):
        if (p[0] > -0.7 and p[0] < -0.1 and p[1] > -0.3 and p[1] < 0.3
                and p[2] > -0.6 - 0.046825 and p[2] < 0 - 0.046825):
            quitar.append(i)
    for i in sorted(quitar, reverse=True):
        del lista[i]
    return np.array(lista)


def original_imu(g, a, cal):
    x, y, z = g[0], -g[1], -g[2]
    th = 15.1 / 180 * 3.1415926
    x2 = np.cos(th) * x - np.sin(th) * z
    y2 = y
    z2 = np.sin(th) * x + np.cos(th) * z
    x2 -= cal["ang_bias_x"]
    y2 -= cal["ang_bias_y"]
    z2 -= cal["ang_bias_z"]
    x2 += cal["ang_z2x_proj"] * z2
    y2 += cal["ang_z2y_proj"] * z2
    ax, ay, az = a[0], -a[1], -a[2]
    ax2 = np.cos(th) * ax - np.sin(th) * az
    az2 = np.sin(th) * ax + np.cos(th) * az
    return (x2, y2, z2), (ax2 - cal["acc_bias_x"], ay - cal["acc_bias_y"], az2 - cal["acc_bias_z"])


def main():
    rng = np.random.default_rng(1)
    # nube con puntos lejos y MUCHOS en la caja del cuerpo (patas)
    n = 20000
    xyz = rng.uniform(-1.5, 1.5, (n, 3))
    ex = np.c_[xyz, rng.uniform(0, 255, n), rng.integers(0, 18, n), rng.uniform(0, 0.1, n)]
    ref = original_nube(ex)
    p, ok = T.transformar_xyz(ex[:, :3])
    mio = np.c_[p[ok], ex[ok, 3:]]
    assert ref.shape == mio.shape, (ref.shape, mio.shape)
    assert np.allclose(ref, mio, atol=1e-9), np.abs(ref - mio).max()
    print("nube: %d puntos, %d del cuerpo quitados, igual a CMU (max dif %.2e)"
          % (n, n - ok.sum(), np.abs(ref - mio).max()))

    cal = {"acc_bias_x": 0.01, "acc_bias_y": -0.02, "acc_bias_z": 0.3, "ang_bias_x": 0.001,
           "ang_bias_y": -0.002, "ang_bias_z": 0.003, "ang_z2x_proj": 0.12, "ang_z2y_proj": -0.25}
    for _ in range(500):
        g, a = rng.normal(0, 1, 3), rng.normal(0, 5, 3)
        r_g, r_a = original_imu(g, a, cal)
        m_g, m_a = T.transformar_imu(g, a, cal)
        assert np.allclose(r_g, m_g) and np.allclose(r_a, m_a)
    print("imu: igual a CMU")

    if Rotation is not None:
        for _ in range(200):
            q = Rotation.random(random_state=int(rng.integers(1e6))).as_quat()
            ref = (Rotation.from_quat(T.Q_IMU) * Rotation.from_quat(q)).as_quat()
            mio = np.array(T.orientacion_imu(tuple(q)))
            assert np.allclose(ref, mio) or np.allclose(ref, -mio)
        e = Rotation.from_euler("xyz", [0.3, -0.2, 1.1]).as_quat()
        assert np.allclose(e, T.quat_desde_euler(0.3, -0.2, 1.1)) or np.allclose(e, -np.array(T.quat_desde_euler(0.3, -0.2, 1.1)))
        print("cuaterniones: igual a scipy / tf_transformations")
    # nivel BYTES, con el formato real del L1: x y z intensity (float32) ring (uint16) time (float32)
    dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": [np.float32] * 4 + [np.uint16, np.float32],
                   "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 32})
    a = np.zeros(n, dt)
    a["x"], a["y"], a["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    a["intensity"] = ex[:, 3]
    a["ring"] = ex[:, 4]
    a["time"] = ex[:, 5]
    datos, k = T.transformar_nube_bytes(a.tobytes(), n, 32, [0, 4, 8])
    b = np.frombuffer(datos, dtype=dt, count=k)
    ref = original_nube(np.c_[a["x"], a["y"], a["z"], a["intensity"], a["ring"], a["time"]].astype(np.float64))
    assert k == len(ref), (k, len(ref))
    assert np.allclose(np.c_[b["x"], b["y"], b["z"]], ref[:, :3], atol=1e-5)
    assert np.array_equal(b["ring"], ref[:, 4].astype(np.uint16)) and np.allclose(b["time"], ref[:, 5])
    import time
    t0 = time.perf_counter()
    for _ in range(20):
        T.transformar_nube_bytes(a.tobytes(), n, 32, [0, 4, 8])
    print("bytes: formato del L1 igual a CMU; %.1f ms por nube de %d puntos" % ((time.perf_counter() - t0) / 20 * 1000, n))
    # sin archivo de calibracion -> los valores de CMU
    d, ok = T.leer_calibracion("/no/existe.yaml")
    assert not ok and d["ang_z2x_proj"] == 0.15 and math.isclose(d["ang_z2y_proj"], -0.28)
    # la caja del perro hasta +0.05 delante del L1 (zona ciega 0.3): una pata de adelante (x -0.02, z -0.35
    # en 'body') se saca; un pie de una persona 15 cm mas adelante (x 0.15) queda
    inv = T.R_NUBE                                      # body = R * L1  ->  L1 = R^T * body
    pata = (np.array([[-0.02, 0.12, -0.35 + T.CAM_OFFSET * 0]]) + [0, 0, T.CAM_OFFSET]) @ inv
    pie = (np.array([[0.15, 0.0, -0.35]]) + [0, 0, T.CAM_OFFSET]) @ inv
    _, ok_cmu = T.transformar_xyz(np.vstack([pata, pie]))
    _, ok_v2 = T.transformar_xyz(np.vstack([pata, pie]), caja_x_max=0.05)
    assert ok_cmu[0] and not ok_v2[0] and ok_v2[1], (ok_cmu, ok_v2)
    print("OK")


if __name__ == "__main__":
    main()

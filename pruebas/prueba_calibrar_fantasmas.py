"""El nodo go2_calibrar_fantasmas entero con ROS falso: se levanta, gira una vuelta (manual de CMU, como el
teclado), frena, calibra, guarda y avisa a go2_fantasmas. python3 pruebas/prueba_calibrar_fantasmas.py"""
import math
import os
import sys
import tempfile
import types

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.argv = ["x"]
import prueba_nodos_sin_ros as F  # noqa: E402  (instala el ROS falso)

ser = types.ModuleType("rclpy.serialization")
ser.deserialize_message = lambda m, t: m
sys.modules["rclpy.serialization"] = ser
sys.modules["std_srvs.srv"].Trigger.Request = F.Msg


class Reloj(object):
    t = 100.0


def odom(yaw):
    o = F.Msg()
    o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = 0.0, 0.0, 0.4
    o.pose.pose.orientation.x = o.pose.pose.orientation.y = 0.0
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return o


def nube(rng):
    pared = np.c_[np.full(300, 2.0), np.linspace(-2, 2, 300), np.linspace(0.3, 1.0, 300)]
    ruido = np.c_[rng.uniform(-3, 3, (20, 2)), rng.uniform(0.3, 1.0, 20)]
    xyz = np.vstack([pared + rng.normal(0, 0.01, pared.shape), ruido]).astype(np.float32)
    a = np.zeros((len(xyz), 16), np.uint8)
    a[:, :12] = xyz.view(np.uint8).reshape(-1, 12)
    import array
    b = array.array("B")
    b.frombytes(a.tobytes())
    m = F.Msg()
    m.width, m.height, m.point_step, m.data = len(xyz), 1, 16, b
    m.fields = [F.Msg(name=k, offset=o) for k, o in zip(("x", "y", "z"), (0, 4, 8))]
    return m


def main():
    from go2_v2 import calibrar_fantasmas as C
    C.time = types.SimpleNamespace(monotonic=lambda: Reloj.t, strftime=lambda f: "2026-10-26 19:30",
                                   sleep=lambda s: None)
    d = tempfile.mkdtemp()
    arch = os.path.join(d, "fantasmas.yaml")
    n = C.Calibrar()
    n.archivo = arch
    rng = np.random.default_rng(0)
    yaw = 0.0
    joys = n.pubs["/joy"].salida
    for k in range(600):                         # 60 s a 10 Hz
        Reloj.t += 0.1
        if n.estado == "girando":
            yaw += n.giro * math.radians(60) * 0.1  # gira como lo manda
        n.subs["/state_estimation"](odom(yaw))
        n.subs["/registered_scan"](nube(rng))
        n.paso()
        n.mandar_giro()
        if n.terminado:
            break
    assert n.terminado and n.estado == "fin", n.estado
    assert n.cli_levantar.llamadas, "no levanto al perro"
    giros = [j for j in joys if j.axes[0] != 0.0]
    assert giros and all(j.header.frame_id == "teclado" and j.axes[5] == -1.0 for j in giros)
    assert joys[-1].axes[0] == 0.0 and joys[-1].axes[5] == 1.0          # al final: suelto (quieto)
    assert n.girado >= 2 * math.pi - 0.1 and n.girado < 2 * math.pi + 0.5, n.girado
    assert n.giro == 0.75
    import yaml
    r = yaml.safe_load(open(arch))
    assert r["min_puntos"] >= 2 and r["guarda_real"] >= 0.97 and r["quita_fantasmas"] > 0.8, r
    assert n.cli_recargar.llamadas, "no le aviso a go2_fantasmas"
    print("calibrar fantasmas (nodo): vuelta de %.0f grados, %d nubes -> min_puntos %d, ventana %.2f"
          % (math.degrees(n.girado), r["nubes"], r["min_puntos"], r["ventana"]))
    print("OK calibrar fantasmas")


if __name__ == "__main__":
    main()

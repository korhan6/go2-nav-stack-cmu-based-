"""Metas directo / FAR, sin ROS: python3 pruebas/prueba_metas.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.metas import Metas, MetasPropio, linea_libre  # noqa: E402
from go2_v2.ruta import Grilla  # noqa: E402


def pared(x, y0, y1, n=40):
    return np.c_[np.full(n, x), np.linspace(y0, y1, n)]


def main():
    vacio = np.zeros((0, 2))
    # linea libre / tapada
    assert linea_libre((0, 0), (2, 0), vacio)
    assert not linea_libre((0, 0), (2, 0), pared(1.0, -1, 1))
    assert linea_libre((0, 0), (2, 0), pared(1.0, 0.5, 1))          # pared al costado: libre
    assert linea_libre((0, 0), (2, 0), np.array([[1.0, 0.0]]))      # 1 punto suelto no tapa
    assert linea_libre((0, 0), (2, 0), pared(2.1, -1, 1))           # meta contra la pared: se llega

    m = Metas(directo_max=3.0)
    # meta cerca y libre -> directo, /way_point = meta
    assert m.nueva_meta((1.9, -0.7, 0), (0, 0, 0), vacio) == [("way", (1.9, -0.7, 0))] and m.modo == "directo"
    # en directo, FAR no manda
    assert m.way_de_far((5, 5, 0)) == []
    # repite la meta
    assert m.paso((0.5, -0.2, 0), vacio) == [("way", (1.9, -0.7, 0))]
    # meta detras de una pared -> FAR
    assert m.nueva_meta((2.0, 0, 0), (0, 0, 0), pared(1.0, -1, 1)) == [("far", (2.0, 0, 0))] and m.modo == "far"
    assert m.way_de_far((0.5, 1.2, 0)) == [("way", (0.5, 1.2, 0))]
    assert m.paso((0, 0, 0), pared(1.0, -1, 1)) == []
    # rodeo la pared con FAR: desde (1.5, 1.5) la linea a la meta ya esta libre -> directo
    assert m.paso((1.5, 1.5, 0), pared(1.0, -1, 1)) == [("way", (2.0, 0, 0))] and m.modo == "directo"
    # meta lejos -> FAR aunque este libre
    assert m.nueva_meta((8, 0, 0), (0, 0, 0), vacio)[0][0] == "far"
    # directo y se tapa la linea lejos de la meta -> vuelve a FAR
    m.nueva_meta((2.5, 0, 0), (0, 0, 0), vacio)
    assert m.modo == "directo"
    assert m.paso((0, 0, 0), pared(1.2, -1, 1)) == [("far", (2.5, 0, 0))] and m.modo == "far"
    # FAR callado (no encuentra ruta): a los 4 s se queda quieto donde esta y avisa
    m = Metas(directo_max=3.0, espera_far=4.0)
    assert m.nueva_meta((2.0, 0, -0.6), (0, 0, 0), pared(1.0, -1, 1), t=10.0)[0][0] == "far"
    assert m.paso((0, 0, 0), pared(1.0, -1, 1), t=13.0) == []
    o = m.paso((0.1, 0.2, 0), pared(1.0, -1, 1), t=14.5)
    assert o == [("way", (0.1, 0.2, 0))] and m.modo == "sin_ruta" and m.avisos, (o, m.modo)
    assert m.way_de_far((1, 1, 0), t=15) == [] and m.paso((0, 0, 0), pared(1.0, -1, 1), t=16) == []
    # FAR hablando: no se corta
    m = Metas(espera_far=4.0)
    m.nueva_meta((8, 0, 0), (0, 0, 0), vacio, t=0.0)
    for k in range(20):
        m.way_de_far((1, 0, 0), t=k * 0.5)
        m.paso((0, 0, 0), pared(1.0, -1, 1), t=k * 0.5 + 0.1)
    assert m.modo == "far"
    # sin pose todavia: FAR
    assert Metas().nueva_meta((1, 0, 0), None, vacio)[0][0] == "far"
    # planificador propio: cuarto cerrado con puerta en x=[1.0, 1.8], meta del otro lado
    def piso(x0, x1, y0, y1):
        xs, ys = np.meshgrid(np.arange(x0, x1, 0.05), np.arange(y0, y1, 0.05))
        return np.c_[xs.ravel(), ys.ravel(), np.zeros(xs.size), np.zeros(xs.size)]

    def muro(x0, y0, x1, y1):
        n = int(np.hypot(x1 - x0, y1 - y0) / 0.03) + 1
        return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, 0.5), np.full(n, 1.0)]
    g = Grilla(res=0.1, lado=30.0)
    g.actualizar(np.vstack([piso(-2, 4, -2, 5), muro(-2, 2, 1.0, 2), muro(1.8, 2, 4, 2), muro(-2, -2, -2, 5),
                            muro(4, -2, 4, 5), muro(-2, -2, 4, -2), muro(-2, 5, 4, 5)]), 0.0)
    mp = MetasPropio(g, adelante=1.2)
    mp.nueva_meta((-1.0, 3.5, 0.0), t=0.0)
    o = mp.paso((-1.0, 0.0, 0.0), t=0.2)
    assert o and o[0][0] == "way", o
    w = o[0][1]
    assert w[0] > -1.0 and w[1] > 0.0, w                   # sale hacia la puerta (a la derecha y arriba)
    # meta en un lugar inalcanzable (dentro de la pared de arriba): va lo mas cerca y avisa una vez
    mp.nueva_meta((-1.0, 5.3, 0.0), t=1.0)
    mp.paso((-1.0, 0.0, 0.0), t=1.2)
    mp.paso((-1.0, 0.0, 0.0), t=1.4)
    assert len(mp.avisos) == 1 and "no se alcanza" in mp.avisos[0], mp.avisos
    assert mp.camino[-1][1] > 4.0
    # LLEGA de frente (meta a 0.3 m, mirando hacia ella): queda quieto donde esta (way_point = el perro)
    mp.nueva_meta((-1.0, 0.3, 0.0), t=2.0)
    assert mp.paso((-1.0, 0.0, 0.0), t=2.2, yaw=math.pi / 2) == [("way", (-1.0, 0.0, 0.0))] and mp.modo == "llego"
    assert mp.paso((-0.9, 0.1, 0.0), t=2.4) == [] and mp.modo == "llego"
    assert mp.paso((0.5, 0.0, 0.0), t=2.6) and mp.modo == "propio"       # lo alejaron: vuelve
    # meta de frente a 0.45 m: todavia no llego (sigue hasta ~0.35)
    mp.nueva_meta((-1.0, 0.45, 0.0), t=2.7)
    o = mp.paso((-1.0, 0.0, 0.0), t=2.8, yaw=math.pi / 2)
    assert mp.modo == "propio" and o and o[0][1] != (-1.0, 0.0, 0.0), o
    # la MISMA distancia pero la meta al COSTADO (90 grados): se queda ahi, sin girar hacia ella
    mp.nueva_meta((-1.0, 0.45, 0.0), t=2.85)
    assert mp.paso((-1.0, 0.0, 0.0), t=2.9, yaw=0.0) == [("way", (-1.0, 0.0, 0.0))] and mp.modo == "llego"
    # al costado pero lejos (0.8 m): sigue yendo
    mp.nueva_meta((-1.0, 0.8, 0.0), t=2.92)
    mp.paso((-1.0, 0.0, 0.0), t=2.95, yaw=0.0)
    assert mp.modo == "propio"
    # el way_point no tiembla: la misma ruta con la pose 5 cm corrida no manda otro
    mp.nueva_meta((-1.0, 3.5, 0.0), t=3.0)
    assert mp.paso((-1.0, 0.0, 0.0), t=3.2)
    assert mp.paso((-0.95, 0.03, 0.0), t=3.4) == []
    print("OK metas")


if __name__ == "__main__":
    main()

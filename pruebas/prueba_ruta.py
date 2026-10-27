"""Planificador propio (A*), sin ROS: python3 pruebas/prueba_ruta.py"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.ruta import LIBRE, OBSTACULO, Grilla, largo, punto_adelante  # noqa: E402


def piso(x0, x1, y0, y1, paso=0.05, z=0.0):
    xs, ys = np.meshgrid(np.arange(x0, x1, paso), np.arange(y0, y1, paso))
    return np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, z), np.zeros(xs.size)]


def pared(x0, y0, x1, y1, paso=0.03, alto=1.0):
    n = int(math.hypot(x1 - x0, y1 - y0) / paso) + 1
    return np.c_[np.linspace(x0, x1, n), np.linspace(y0, y1, n), np.full(n, alto / 2), np.full(n, alto)]


def main():
    # cuarto de 4x4 con una pared en y=2 y una PUERTA de 0.8 m en x=[1.0, 1.8]; meta del otro lado
    g = Grilla(res=0.1, lado=30.0)
    caja = [pared(-2, -2, -2, 5), pared(4, -2, 4, 5), pared(-2, -2, 4, -2), pared(-2, 5, 4, 5)]
    nube = np.vstack([piso(-2, 4, -2, 5), pared(-2, 2, 1.0, 2), pared(1.8, 2, 4, 2)] + caja)
    g.actualizar(nube, piso_z=0.0)
    t0 = time.perf_counter()
    c = g.planificar((-1.0, 0.0), (-1.0, 3.5))
    ms = (time.perf_counter() - t0) * 1000
    assert c is not None
    cruce = [p for p in c if 1.9 <= p[1] <= 2.1]
    assert cruce and all(1.0 < p[0] < 1.8 for p in cruce), cruce          # cruza POR la puerta
    assert all(abs(p[0] - 1.4) < 0.25 for p in cruce), cruce              # por el centro de la puerta
    assert largo(c) < 7.5, largo(c)                                       # y no da vueltas
    w = punto_adelante(c, 1.2)
    assert abs(math.hypot(w[0] + 1.0, w[1]) - 1.2) < 0.15
    # puerta de 0.4 m: no cabe (radio 0.25 m) -> sin camino
    g2 = Grilla(res=0.1, lado=30.0)
    g2.actualizar(np.vstack([piso(-2, 4, -2, 5), pared(-2, 2, 1.0, 2), pared(1.4, 2, 4, 2),
                             pared(-2, -2, -2, 5), pared(4, -2, 4, 5), pared(-2, -2, 4, -2), pared(-2, 5, 4, 5)]), 0.0)
    assert g2.planificar((-1.0, 0.0), (-1.0, 3.5)) is None
    # ...pero lo_mas_cerca lo lleva hasta la puerta (lo mas cerca de la meta que se puede)
    c2 = g2.planificar((-1.0, 0.0), (-1.0, 3.5), lo_mas_cerca=True)
    assert c2 is not None and 1.4 < c2[-1][1] < 2.0 and math.hypot(c2[-1][0] + 1.0, c2[-1][1] - 3.5) < 2.2, c2[-1]
    # piso mas bajo (0.6 m): ya NO es obstaculo (26/10: los 'huecos' marcaban negro medio cuarto)
    g3 = Grilla(res=0.1, lado=30.0)
    g3.actualizar(np.vstack([piso(-2, 4, -2, 4), piso(0.5, 1.5, -2, 2.5, z=-0.6)]), 0.0)
    c3 = g3.planificar((0.0, 0.0), (2.5, 0.0))
    assert c3 is not None and largo(c3) < 3.0, c3
    assert (g3.c != OBSTACULO).all()
    # lo que se movio se limpia: una persona que se fue
    g4 = Grilla(res=0.1, lado=30.0)
    g4.actualizar(np.vstack([piso(-1, 3, -1, 1), pared(1.0, -0.5, 1.0, 0.5)]), 0.0)
    assert g4.c[g4.celda(1.0, 0.0)] == OBSTACULO
    g4.actualizar(piso(0.5, 1.5, -1, 1), 0.0)
    assert g4.c[g4.celda(1.0, 0.0)] == LIBRE
    # meta pegada a la pared: va a la celda libre mas cercana
    c5 = g.planificar((-1.0, 0.0), (-1.0, 1.95))
    assert c5 is not None and math.hypot(c5[-1][0] + 1.0, c5[-1][1] - 1.95) < 0.6
    # el perro arranca pegado a la pared: igual sale
    assert g.planificar((-1.0, 1.85), (3.0, 0.0)) is not None
    # PISADAS: un fantasma en (1.0, 0) bloquea; el perro lo atraviesa (teleop) -> se borra y no vuelve
    g6 = Grilla(res=0.1, lado=30.0)
    fantasma = np.c_[np.full(20, 1.0), np.linspace(-0.1, 0.1, 20), np.full(20, 0.3), np.full(20, 0.4)]
    g6.actualizar(np.vstack([piso(-1, 3, -1, 1), fantasma]), 0.0)
    assert g6.c[g6.celda(1.0, 0.0)] == OBSTACULO
    nube = np.vstack([piso(-1, 3, -1, 1), fantasma])
    for k in range(30):                              # camina de x=0 a x=2 por y=0, con el mapa de ahora
        g6.pisar(k * 0.07, 0.0, 0.0, nube=nube)
    assert g6.c[g6.celda(1.0, 0.0)] == LIBRE
    ok = g6.filtrar_pisadas(nube)
    assert not ok[-20:].any() and ok[:-20].all()     # el fantasma se saca, el piso queda
    g6.actualizar(nube[ok], 0.0)
    assert g6.c[g6.celda(1.0, 0.0)] == LIBRE         # no vuelve
    alto = np.array([[1.0, 0.0, 1.2, 1.2]])          # algo ALTO ahi (una persona): se sigue viendo
    assert g6.filtrar_pisadas(alto).all()
    costado = np.array([[1.0, 0.6, 0.3, 0.4]])        # lo que esta al costado (no pisado): se queda
    assert g6.filtrar_pisadas(costado).all()
    # guardar / cargar (con pisadas)
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        f = os.path.join(d, "g.npz")
        g6.guardar(f)
        g7 = Grilla(res=0.1, lado=30.0)
        g7.cargar(f)
        assert (g7.c == g6.c).all() and (g7.pisadas == g6.pisadas).all() and (g7.fantasmas == g6.fantasmas).all()
        try:
            Grilla(res=0.1, lado=20.0).cargar(f)
            raise AssertionError("tenia que fallar")
        except ValueError:
            pass
    # una PERSONA que se para DESPUES en un lugar por donde el perro ya camino (x=1.5, y=0): se ve
    # (26/10: antes sus piernas se borraban y el perro se le iba encima)
    persona = np.c_[np.full(15, 1.5), np.linspace(-0.08, 0.08, 15), np.linspace(0.1, 0.6, 15), np.linspace(0.15, 0.65, 15)]
    assert g6.pisadas[g6.celda(1.5, 0.0)] and not g6.fantasmas[g6.celda(1.5, 0.0)]
    assert g6.filtrar_pisadas(persona).all()
    g6.actualizar(np.vstack([piso(-1, 3, -1, 1), persona]), 0.0)
    assert g6.c[g6.celda(1.5, 0.0)] == OBSTACULO
    # sin nube al pisar (no se sabe que habia): no se confirma nada
    g8 = Grilla(res=0.1, lado=30.0)
    g8.pisar(1.0, 0.0, 0.0)
    assert g8.pisadas.any() and not g8.fantasmas.any()
    # lo que esta pegado al COSTADO del cuerpo (no debajo) no es fantasma
    g9 = Grilla(res=0.1, lado=30.0)
    al_lado = np.array([[1.0, 0.17, 0.2, 0.3]])
    g9.pisar(1.0, 0.0, 0.0, nube=al_lado)
    assert not g9.fantasmas.any()
    g6.olvidar_pisadas()
    assert g6.filtrar_pisadas(nube).all()
    print("ruta: cruza por el centro de la puerta (%.1f m, %.0f ms), no pasa si no cabe" % (largo(c), ms))
    print("OK ruta")


if __name__ == "__main__":
    main()

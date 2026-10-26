"""Filtro de fantasmas y mapa, sin ROS: python3 pruebas/prueba_filtros.py"""
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.filtros import (AcumuladorMapa, FiltroFantasmas, codificar, decodificar,  # noqa: E402
                            escribir_pcd, xyz_de_nube)


def nube_real(rng, n_suelo=3000):
    """Piso a 0 m (6x6 m), una pared a x=2 m y una pata de silla fina (3 cm) en (1.2, 0.5)."""
    suelo = np.c_[rng.uniform(-3, 3, n_suelo), rng.uniform(-3, 3, n_suelo), rng.normal(0, 0.01, n_suelo)]
    pared = np.c_[np.full(400, 2.0) + rng.normal(0, 0.01, 400), rng.uniform(-1, 1, 400), rng.uniform(0, 1, 400)]
    # la pata se ve con pocos puntos por nube (lo dificil: no confundirla con ruido)
    pata = np.c_[1.2 + rng.normal(0, 0.01, 3), 0.5 + rng.normal(0, 0.01, 3), rng.uniform(0.05, 0.45, 3)]
    return suelo, pared, pata


def main():
    rng = np.random.default_rng(1)
    # claves: ida y vuelta, negativos incluidos
    ijk = np.array([[0, 0, 0], [-5, 7, -1], [1000, -1000, 3]])
    assert (decodificar(codificar(ijk)) == ijk).all()

    f = FiltroFantasmas(tam=0.1, min_puntos=5, ventana=0.5, radio=4.0)
    fantasmas_sobrevivientes = 0
    fantasmas_total = 0
    pata_vista = []
    for k in range(10):                                    # 1 s de nubes a 10 Hz
        suelo, pared, pata = nube_real(rng)
        # fantasmas en el aire delante del perro (lejos de la pared y de la pata: pegados a algo real
        # no se pueden distinguir, ni hace falta)
        ruido = np.c_[rng.uniform(0.3, 1.7, 10), rng.uniform(-1, 0.2, 10), rng.uniform(0.3, 0.8, 10)]
        lejos = np.array([[8.0, 0.0, 0.5]])                # punto suelto lejos: no se toca
        xyz = np.vstack([suelo, pared, pata, ruido, lejos])
        t0 = time.perf_counter()
        ok = f.filtrar(k * 0.1, xyz, (0.0, 0.0, 0.4))     # lidar a 0.4 m del piso
        ms = (time.perf_counter() - t0) * 1000
        i0 = len(suelo)
        i1 = i0 + len(pared)
        i2 = i1 + len(pata)
        i3 = i2 + len(ruido)
        assert ok[:i0].all()                                     # el piso NUNCA se filtra
        assert ok[i0:i1].mean() > 0.97, ok[i0:i1].mean()          # la pared queda
        assert ok[i3], "lo que esta lejos no se filtra"
        if k >= 2:
            pata_vista.append(ok[i1:i2].mean())
            fantasmas_total += len(ruido)
            fantasmas_sobrevivientes += ok[i2:i3].sum()
    tasa = fantasmas_sobrevivientes / float(fantasmas_total)
    assert tasa < 0.1, tasa                                # casi todos los fantasmas afuera
    assert np.mean(pata_vista) > 0.9, pata_vista           # la pata fina de silla SE QUEDA
    assert ms < 50, ms
    print("filtro: fantasmas que pasan %.1f%%, pata de silla conservada %.0f%%, %.1f ms por nube"
          % (100 * tasa, 100 * np.mean(pata_vista), ms))

    # mapa: voxeles unicos, version liviana, PCD
    m = AcumuladorMapa(tam=0.05)
    m.agregar(np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [1.0, 1.0, 1.0]]))
    m.agregar(np.array([[0.03, 0.03, 0.03]]))
    assert m.consolidar() == 2
    p = m.puntos()
    assert len(p) == 2 and np.allclose(sorted(p[:, 0]), [0.025, 1.025])
    assert len(m.puntos(tam_salida=10.0)) == 1
    with tempfile.TemporaryDirectory() as d:
        ruta = os.path.join(d, "m.pcd")
        assert escribir_pcd(ruta, p) == 2
        datos = open(ruta, "rb").read()
        cab, cuerpo = datos.split(b"DATA binary\n")
        assert b"POINTS 2" in cab and len(cuerpo) == 2 * 12
        assert np.allclose(np.frombuffer(cuerpo, np.float32).reshape(2, 3), p)
    m.borrar()
    assert m.consolidar() == 0

    # lectura de nube: x y z en offsets cualquiera (PointXYZINormal de Point-LIO: paso 48)
    xyz = rng.normal(size=(5, 3)).astype(np.float32)
    raw = np.zeros((5, 48), np.uint8)
    for i in range(3):
        raw[:, 4 * i:4 * i + 4] = xyz[:, i:i + 1].copy().view(np.uint8)
    leido, _ = xyz_de_nube(raw.tobytes(), 5, 48, [0, 4, 8])
    assert np.allclose(leido, xyz)
    print("OK filtros")


if __name__ == "__main__":
    main()

"""Control PS4 -> /joy de CMU, sin ROS: python3 pruebas/prueba_ps4.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.ps4 import Modo, mapear  # noqa: E402


def cmu(a):
    """Lo que hace el pathFollower de CMU con esos ejes (modo manual: sin evasion)."""
    manual = a[5] < -0.1
    autonomo = a[2] < -0.1
    return manual, autonomo


def main():
    suelto = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    # sin evasion: stick izquierdo adelante -> manual de CMU, adelante
    a = list(suelto)
    a[1] = 0.8
    o = mapear(a, True)
    assert cmu(o) == (True, False) and o[4] == 0.8 and o[0] == 0.0 and o[3] == 0.0
    # girar con el izquierdo, de lado con el derecho
    a = list(suelto)
    a[0] = -0.5
    a[3] = 0.6
    o = mapear(a, True)
    assert cmu(o)[0] and o[0] == -0.5 and o[3] == 0.6 and o[4] == 0.0
    # sticks sueltos: ni manual ni autonomo -> el perro se queda quieto
    assert cmu(mapear(suelto, True)) == (False, False)
    assert mapear(suelto, True)[4] == 0.0
    # zona muerta
    a = list(suelto)
    a[1] = 0.05
    assert not cmu(mapear(a, True))[0]
    # L2 sostenido: autonomo, pasa tal cual
    a = list(suelto)
    a[2] = -1.0
    a[1] = 0.9
    assert mapear(a, True) == a
    # con evasion: tal cual (CMU: stick derecho esquivando)
    a = list(suelto)
    a[4] = 0.7
    assert mapear(a, False) == a
    # OPTIONS cambia de modo (flanco), sostenerlo no repite
    m = Modo(True)
    b = [0] * 13
    b[9] = 1
    assert m.boton(b) and not m.sin_evasion
    assert not m.boton(b) and not m.sin_evasion
    assert not m.boton([0] * 13)
    assert m.boton(b) and m.sin_evasion
    print("OK ps4")


if __name__ == "__main__":
    main()

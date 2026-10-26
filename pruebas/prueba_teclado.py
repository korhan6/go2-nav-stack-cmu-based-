"""Teclado -> /joy de CMU, sin ROS: python3 pruebas/prueba_teclado.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.teclado import Teclado, ejes, quieto  # noqa: E402


def modo_cmu(a):
    """Lo que hace CMU con esos ejes (localPlanner/pathFollower: joystickHandler)."""
    autonomo = not (a[2] > -0.1)
    manual = not (a[5] > -0.1)
    vel = min(1.0, (a[3] ** 2 + a[4] ** 2) ** 0.5) if a[4] != 0 else 0.0
    if a[4] < 0:
        vel = 0.0                        # twoWayDrive=false: con evasion no va atras
    return autonomo, manual, vel


def main():
    # W: con evasion, adelante, nunca modo autonomo
    au, ma, v = modo_cmu(ejes("w", 0.5))
    assert not au and not ma and abs(v - 0.5) < 1e-9
    # Q / E: con evasion, curva a la izquierda / derecha (stick der. X: izquierda +)
    assert ejes("q", 0.5)[3] > 0 and ejes("e", 0.5)[3] < 0 and not modo_cmu(ejes("q", 0.5))[1]
    # A / D: manual, giro izquierda (+) / derecha (-), sin avanzar
    a = ejes("a", 0.5)
    assert modo_cmu(a)[1] and a[0] > 0 and a[4] == 0
    assert ejes("d", 0.5)[0] < 0
    # S: manual (con evasion CMU no deja ir atras), hacia atras
    s = ejes("s", 0.5)
    assert modo_cmu(s)[1] and s[4] < 0
    # Z / C: de lado, manual
    assert modo_cmu(ejes("z", 0.5))[1] and ejes("z", 0.5)[3] > 0 and ejes("c", 0.5)[3] < 0
    # quieto: ni autonomo ni manual, velocidad 0
    assert modo_cmu(quieto()) == (False, False, 0.0)
    assert ejes("x", 0.5) is None

    tk = Teclado(v=0.5, suelta=0.6)
    assert tk.apretar(0.0, "w") is None and tk.ahora(0.0) == ejes("w", 0.5)
    assert tk.ahora(0.5) == ejes("w", 0.5)          # repeticion del teclado todavia no llego: sigue
    assert tk.ahora(0.7) == quieto()                 # se solto
    tk.apretar(1.0, "W")                              # mayuscula tambien
    assert tk.ahora(1.0) == ejes("w", 0.5)
    assert tk.apretar(1.1, " ") == "frenar" and tk.ahora(1.1) == quieto()
    assert tk.apretar(1.2, "+") == "vel" and tk.v == 0.6
    for _ in range(10):
        tk.apretar(1.3, "+")
    assert tk.v == 1.0
    for _ in range(20):
        tk.apretar(1.3, "-")
    assert tk.v == 0.1
    assert tk.apretar(2, "g") == "mapa" and tk.apretar(2, "r") == "fantasmas" and tk.apretar(2, "p") == "pausa"
    assert tk.apretar(2, "\x1b") == "salir"
    print("OK teclado")


if __name__ == "__main__":
    main()

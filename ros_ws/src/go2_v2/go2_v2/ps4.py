"""go2_ps4: el control PS4 para manejar al perro SIN evasion de obstaculos (por defecto).

  joy_node (/joy_ps4) -> /joy (lo que leen CMU y el guardian)

  Stick IZQUIERDO: adelante / atras / derecha / izquierda (de lado). Stick DERECHO: girar.
  SIN EVASION (por defecto): va directo al perro (modo manual de CMU, como R2 sostenido): no esquiva
     nada, manejas tu.
  CON EVASION (OPTIONS cambia): los mismos sticks, y el planificador de CMU esquiva solo.
  L2 sostenido: sigue la meta (autonomo), en los dos modos. Botones (los atiende go2_guardian):
  CRUZ acostar, TRIANGULO levantar, CIRCULO pausa, R1 borra el mapa de terreno, L1 el grafo de FAR.
"""
import sys

SUELTO = 1.0
APRETADO = -1.0
ZONA = 0.12          # zona muerta de los sticks (la misma del joy_node)
BOTON_OPTIONS = 9


def mapear(ejes, sin_evasion):
    """Ejes del PS4 (joy_node: 0 izq X, 1 izq Y, 2 L2, 3 der X, 4 der Y, 5 R2) -> los que entiende CMU
    (0 giro, 2 L2 autonomo, 3 lateral, 4 adelante, 5 R2 manual). Stick izquierdo = moverse (adelante,
    atras, de lado), stick derecho (X) = girar."""
    a = list(ejes) + [0.0] * max(0, 8 - len(ejes))
    if a[2] < -0.1:                                  # L2 (sigue la meta): tal cual
        return a
    izq_x, izq_y, der_x = a[0], a[1], a[3]
    out = list(a)
    out[0] = der_x                                   # girar (stick derecho)
    out[1] = 0.0
    out[3] = izq_x                                   # de lado (stick izquierdo)
    out[4] = izq_y                                   # adelante / atras (stick izquierdo)
    if sin_evasion:
        mueve = max(abs(izq_x), abs(izq_y), abs(der_x)) > ZONA
        out[2] = SUELTO
        out[5] = APRETADO if mueve else SUELTO       # manual de CMU (sin evasion) mientras mueve
    return out


class Modo(object):
    def __init__(self, sin_evasion=True):
        self.sin_evasion = sin_evasion
        self.prev = False

    def boton(self, botones):
        """OPTIONS (flanco de subida) cambia de modo. Devuelve True si cambio."""
        b = len(botones) > BOTON_OPTIONS and bool(botones[BOTON_OPTIONS])
        cambio = b and not self.prev
        self.prev = b
        if cambio:
            self.sin_evasion = not self.sin_evasion
        return cambio


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy

    class Nodo(Node):
        def __init__(self):
            super().__init__("go2_ps4")
            self.modo = Modo(bool(self.declare_parameter("sin_evasion", True).value))
            self.pub = self.create_publisher(Joy, "/joy", 10)
            self.create_subscription(Joy, "/joy_ps4", self.cb, 10)
            self.get_logger().info("PS4: %s (OPTIONS cambia). Stick izq: moverse (adelante/atras/de lado), "
                                   "der: girar. CRUZ acostar, TRIANGULO levantar, CIRCULO pausa"
                                   % ("SIN evasion" if self.modo.sin_evasion else "CON evasion"))

        def cb(self, m):
            if self.modo.boton(list(m.buttons)):
                self.get_logger().warn("PS4: ahora %s" % ("SIN evasion (manejas tu)" if self.modo.sin_evasion
                                                          else "CON evasion (CMU esquiva)"))
            out = Joy()
            out.header = m.header
            out.axes = [float(v) for v in mapear(m.axes, self.modo.sin_evasion)]
            out.buttons = list(m.buttons)
            self.pub.publish(out)

    rclpy.init()
    n = Nodo()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())

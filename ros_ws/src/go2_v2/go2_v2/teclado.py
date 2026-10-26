"""go2_teclado: manejar el perro con el teclado (WASD) mientras se arma el mapa.

Habla con el stack de CMU igual que el control PS4 (/joy), asi que:
  W, Q, E  -> CON evasion: el planificador de CMU elige el camino libre hacia esa direccion
  A, D, S, Z, C -> manual SIN evasion (girar en el sitio, atras, de lado): despacio y mirando
El guardian sigue frenando si falta el lidar o el terreno. Mantener apretada la tecla = camina;
soltarla = se frena (en ~0.6 s, lo que tarda la repeticion del teclado). ESPACIO frena ya.
Manejar con el teclado cancela una meta en curso (igual que mover el stick del PS4).

  go2v2 manejar      (con la v2 corriendo)
"""
import os
import select
import sys
import termios
import time
import tty

AYUDA = """
  go2v2 - manejar con el teclado                      velocidad: %(v)3.0f%%   (+ / - para cambiar)
  --------------------------------------------------------------------------------
     Q   W   E        W  adelante (esquiva)     Q / E  adelante girando (esquiva)
     A   S   D        A / D  girar en el sitio  S  atras (sin evasion: despacio)
     Z       C        Z / C  de lado (sin evasion)
                      ESPACIO  frenar           P  pausa (freno de emergencia)
                      R  borrar fantasmas del mapa de terreno
                      G  guardar el mapa (.pcd en ~/.go2_v2/mapas)
                      ESC o Ctrl+C  salir (el perro queda quieto)
  --------------------------------------------------------------------------------
"""

SUELTO = 1.0          # gatillo L2/R2 suelto (CMU: > -0.1)
APRETADO = -1.0
GIRO = 0.5            # fraccion de la velocidad de giro maxima de CMU (80 grados/s)
LADO = 0.6            # fraccion del stick para ir de lado / la curva de Q y E
ATRAS = 0.5           # fraccion para ir atras (sin evasion)


def ejes(tecla, v):
    """Tecla -> los 8 ejes del /joy que entiende CMU (mapeo PS4 de joy_node):
    [0] stick izq. X (giro en manual) [3] stick der. X (izquierda +) [4] stick der. Y (adelante +)
    [2] L2 (apretado = modo autonomo) [5] R2 (apretado = manual SIN evasion).
    None = tecla que no mueve."""
    a = [0.0, 0.0, SUELTO, 0.0, 0.0, SUELTO, 0.0, 0.0]
    if tecla == "w":
        a[4] = v
    elif tecla == "q":
        a[4], a[3] = v, LADO * v
    elif tecla == "e":
        a[4], a[3] = v, -LADO * v
    elif tecla == "a":
        a[5], a[0] = APRETADO, GIRO
    elif tecla == "d":
        a[5], a[0] = APRETADO, -GIRO
    elif tecla == "s":
        a[5], a[4] = APRETADO, -ATRAS * v
    elif tecla == "z":
        a[5], a[3] = APRETADO, LADO
    elif tecla == "c":
        a[5], a[3] = APRETADO, -LADO
    else:
        return None
    return a


def quieto():
    return [0.0, 0.0, SUELTO, 0.0, 0.0, SUELTO, 0.0, 0.0]


class Teclado(object):
    """Logica sin ROS: que ejes mandar en cada momento."""

    def __init__(self, v=0.5, suelta=0.6):
        self.v = v
        self.suelta = suelta            # s sin repeticion de la tecla = se solto
        self.tecla = None
        self.t_tecla = -1e9

    def apretar(self, t, tecla):
        """Devuelve una accion especial ('salir', 'frenar', 'pausa', 'fantasmas', 'mapa', 'vel') o None."""
        if tecla in ("\x1b", "\x03"):
            self.tecla = None
            return "salir"
        if tecla == " ":
            self.tecla = None
            return "frenar"
        if tecla in ("+", "="):
            self.v = min(1.0, round(self.v + 0.1, 2))
            return "vel"
        if tecla in ("-", "_"):
            self.v = max(0.1, round(self.v - 0.1, 2))
            return "vel"
        t2 = tecla.lower()
        if t2 == "p":
            return "pausa"
        if t2 == "r":
            return "fantasmas"
        if t2 == "g":
            return "mapa"
        if ejes(t2, self.v) is not None:
            self.tecla = t2
            self.t_tecla = t
        return None

    def ahora(self, t):
        if self.tecla is None or t - self.t_tecla > self.suelta:
            self.tecla = None
            return quieto()
        return ejes(self.tecla, self.v)


def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_srvs.srv import Trigger

    if not sys.stdin.isatty():
        print("go2v2 manejar necesita una terminal (no se puede usar con tuberias)")
        return 1
    rclpy.init()
    nodo = Node("go2_teclado")
    pub = nodo.create_publisher(Joy, "/joy", 5)
    cli = {k: nodo.create_client(Trigger, "/go2/" + k) for k in ("pausa", "guardar_mapa")}
    tk = Teclado(v=float(os.environ.get("GO2V2_VEL_TECLADO", "0.5")))

    def publicar(a, botones=None):
        j = Joy()
        j.header.stamp = nodo.get_clock().now().to_msg()
        j.header.frame_id = "teclado"           # el guardian lo ignora (no son combinaciones del PS4)
        j.axes = [float(x) for x in a]
        j.buttons = botones or [0] * 11
        pub.publish(j)

    def llamar(nombre):
        c = cli[nombre]
        if not c.wait_for_service(timeout_sec=1.0):
            return "no esta el servicio /go2/%s (corre la v2 completa?)" % nombre
        fut = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(nodo, fut, timeout_sec=3.0)
        r = fut.result()
        return r.message if r is not None else "sin respuesta"

    viejo = termios.tcgetattr(sys.stdin)
    msg = ""
    try:
        tty.setcbreak(sys.stdin.fileno())
        print(AYUDA % {"v": tk.v * 100})
        ultimo = None
        # quieto se manda solo unas veces al soltar: si se mandara siempre, pisaria la meta que se
        # le de desde RViz o con 'go2v2 ir' (CMU pasa a modo joystick con cada /joy)
        quietos = 0
        while rclpy.ok():
            t = time.monotonic()
            listo, _, _ = select.select([sys.stdin], [], [], 0.05)
            acc = None
            if listo:
                c = sys.stdin.read(1)
                acc = tk.apretar(time.monotonic(), c)
                if acc == "salir":
                    break
                if acc == "vel":
                    msg = "velocidad %d%%" % (tk.v * 100)
                elif acc == "pausa":
                    msg = "pausa: " + llamar("pausa")
                elif acc == "mapa":
                    msg = "mapa: " + llamar("guardar_mapa")
                elif acc == "fantasmas":
                    b = [0] * 11
                    b[5] = 1                     # = R1 del PS4: CMU borra su mapa de terreno
                    publicar(quieto(), b)
                    msg = "mapa de terreno borrado (fantasmas fuera)"
                elif acc == "frenar":
                    msg = "frenado"
            if tk.tecla is not None or acc == "frenar":
                quietos = 5
            a = tk.ahora(time.monotonic())
            if tk.tecla is not None:
                publicar(a)
            elif quietos > 0:
                publicar(a)
                quietos -= 1
            estado = tk.tecla.upper() if tk.tecla else "quieto"
            linea = "\r  %-8s | vel %3d%% | %-60s" % (estado, tk.v * 100, msg[:60])
            if linea != ultimo:
                sys.stdout.write(linea)
                sys.stdout.flush()
                ultimo = linea
            rclpy.spin_once(nodo, timeout_sec=0.0)
            resto = 0.05 - (time.monotonic() - t)       # ~20 Hz
            if resto > 0:
                time.sleep(resto)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        for _ in range(3):
            publicar(quieto())
            time.sleep(0.05)
        print("\n  listo: el perro queda quieto")
        nodo.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

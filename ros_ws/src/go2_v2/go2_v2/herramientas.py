"""Comandos de la v2 (los llama 'go2v2' desde fuera del Docker):

  ir X Y            ir a (X, Y) en metros del mapa (el origen es donde arranco el perro)
  ir NOMBRE         ir a un punto guardado
  guardar NOMBRE    guarda donde esta el perro ahora
  puntos            lista los puntos guardados
  borrar NOMBRE
  parar             corta la navegacion (el perro queda quieto; el control PS4 sigue andando)
  estado [s]        sensores, guardian, pose y demora durante unos segundos
  deriva [s]        con el perro QUIETO: cuanto se corre la pose de Point-LIO en s segundos (20)

Los puntos (~/.go2_v2/puntos.yaml, en el Docker /root/.go2_v2) valen mientras el perro arranque
SIEMPRE desde el mismo lugar y mirando hacia el mismo lado: Point-LIO empieza el mapa donde se
prende. (Para mapas fijos con relocalizacion: siguiente paso de la v2.)
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

ARCHIVO = os.path.expanduser("~/.go2_v2/puntos.yaml")


def leer_puntos():
    try:
        import yaml
        with open(ARCHIVO) as f:
            d = yaml.safe_load(f) or {}
        return {str(k): (float(v[0]), float(v[1])) for k, v in d.items()}
    except (OSError, ValueError, TypeError, KeyError, IndexError):
        return {}
    except Exception:
        return {}


def escribir_puntos(p):
    import yaml
    os.makedirs(os.path.dirname(ARCHIVO), exist_ok=True)
    with open(ARCHIVO, "w") as f:
        yaml.safe_dump({k: [round(v[0], 3), round(v[1], 3)] for k, v in sorted(p.items())}, f)


def joy_navegar():
    """El mismo mensaje que manda el boton 'Goalpoint'/'Resume' de RViz de CMU: modo waypoint."""
    j = Joy()
    j.header.frame_id = "go2v2"
    j.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
    j.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
    return j


def joy_quieto():
    """Modo joystick con el joystick suelto: el planificador frena y deja al perro quieto."""
    j = Joy()
    j.header.frame_id = "go2v2"
    j.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    j.buttons = [0] * 11
    return j


class Herramientas(Node):
    def __init__(self):
        super().__init__("go2v2_cli")
        self.pose = None
        self.estados = {}
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, 5)
        for top in ("/go2/sensores_estado", "/go2/guardian_estado", "/go2/fantasmas_estado", "/go2/metas_estado"):
            self.create_subscription(String, top, lambda m, k=top: self.estados.__setitem__(k, m.data), 5)
        self.pub_goal = self.create_publisher(PointStamped, "/goal_point", 5)
        self.pub_way = self.create_publisher(PointStamped, "/way_point", 5)
        self.pub_joy = self.create_publisher(Joy, "/joy", 5)

    def cb_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.pose = (p.x, p.y, p.z, yaw, st)

    def girar(self, seg, hasta=None):
        t0 = time.monotonic()
        while time.monotonic() - t0 < seg and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if hasta is not None and hasta():
                return True
        return False

    def esperar_suscriptores(self, pub, seg=3.0):
        t0 = time.monotonic()
        while pub.get_subscription_count() == 0 and time.monotonic() - t0 < seg:
            rclpy.spin_once(self, timeout_sec=0.05)
        return pub.get_subscription_count() > 0

    def ir(self, x, y):
        hay_far = self.esperar_suscriptores(self.pub_goal, 3.0)
        self.esperar_suscriptores(self.pub_joy, 1.0)
        st = self.get_clock().now().to_msg()
        joy = joy_navegar()
        joy.header.stamp = st
        self.pub_joy.publish(joy)
        p = PointStamped()
        p.header.stamp = st
        p.header.frame_id = "map"
        p.point.x, p.point.y = float(x), float(y)
        p.point.z = self.pose[2] if self.pose else 0.0
        for _ in range(2):
            if hay_far:
                self.pub_goal.publish(p)            # el planificador de rutas (FAR) arma el camino
            else:
                self.pub_way.publish(p)             # sin FAR: el planificador local va directo
            self.girar(0.1)
        return hay_far


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 0
    cmd = args[0]
    puntos = leer_puntos()
    if cmd == "puntos":
        if not puntos:
            print("no hay puntos guardados (go2v2 guardar NOMBRE)")
        for k, (x, y) in sorted(puntos.items()):
            print("  %-15s x=%7.2f  y=%7.2f" % (k, x, y))
        return 0
    if cmd == "borrar" and len(args) > 1:
        if puntos.pop(args[1], None) is None:
            print("no existe '%s'" % args[1])
            return 1
        escribir_puntos(puntos)
        print("borrado '%s'" % args[1])
        return 0
    rclpy.init()
    n = Herramientas()
    try:
        if cmd == "guardar" and len(args) > 1:
            if not n.girar(5.0, lambda: n.pose is not None):
                print("no llega la pose (/state_estimation): esta corriendo 'go2v2 lanzar'?")
                return 1
            puntos[args[1]] = (n.pose[0], n.pose[1])
            escribir_puntos(puntos)
            print("guardado '%s' en x=%.2f y=%.2f" % (args[1], n.pose[0], n.pose[1]))
            return 0
        if cmd == "ir" and len(args) > 1:
            if len(args) >= 3:
                try:
                    x, y = float(args[1]), float(args[2])
                except ValueError:
                    print("uso: go2v2 ir X Y  |  go2v2 ir NOMBRE")
                    return 1
            elif args[1] in puntos:
                x, y = puntos[args[1]]
            else:
                print("no conozco '%s' (go2v2 puntos)" % args[1])
                return 1
            far = n.ir(x, y)
            print("voy a x=%.2f y=%.2f (%s)" % (x, y, "ruta con FAR planner" if far else
                                               "directo con el planificador local: lanza con ruta:=true para rutas largas"))
            return 0
        if cmd == "parar":
            n.esperar_suscriptores(n.pub_joy, 2.0)
            j = joy_quieto()
            j.header.stamp = n.get_clock().now().to_msg()
            for _ in range(3):
                n.pub_joy.publish(j)
                n.girar(0.05)
            print("navegacion cortada: quieto")
            return 0
        if cmd == "deriva":
            seg = float(args[1]) if len(args) > 1 else 20.0
            if not n.girar(5.0, lambda: n.pose is not None):
                print("no llega la pose (/state_estimation): esta corriendo 'go2v2 lanzar'?")
                return 1
            p0 = n.pose
            print("perro QUIETO, midiendo %.0f s..." % seg)
            n.girar(seg)
            p1 = n.pose
            d = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) * 100.0
            dz = (p1[2] - p0[2]) * 100.0
            dyaw = math.degrees(math.atan2(math.sin(p1[3] - p0[3]), math.cos(p1[3] - p0[3])))
            print("se corrio %.1f cm en el plano, %.1f cm de altura y %.1f grados de rumbo en %.0f s"
                  % (d, dz, dyaw, seg))
            if d < 5 and abs(dyaw) < 2 and abs(dz) < 5:
                print("BIEN: Point-LIO esta firme (lo que se mueve en RViz serian las patas del modelo)")
            elif abs(dyaw) >= 2 and d < 10:
                print("MAL: GIRA solo -> el giroscopio (IMU) tiene sesgo: 'go2v2 calibrar' con el perro de pie")
            else:
                print("MAL: se DESLIZA -> Point-LIO no engancha con la nube (calibracion, o arrancalo quieto 20 s)")
            return 0
        if cmd == "estado":
            seg = float(args[1]) if len(args) > 1 else 3.0
            n.girar(seg)
            ahora = n.get_clock().now().nanoseconds * 1e-9
            if n.pose:
                x, y, z, yaw, st = n.pose
                print("pose (Point-LIO): x=%.2f y=%.2f z=%.2f rumbo=%.0f grados | demora de la odometria %.0f ms"
                      % (x, y, z, math.degrees(yaw), (ahora - st) * 1000))
            else:
                print("pose: NO llega /state_estimation (Point-LIO no arranco: sensores? calibracion?)")
            for k, v in sorted(n.estados.items()):
                try:
                    print("%s: %s" % (k, json.dumps(json.loads(v), ensure_ascii=False)))
                except ValueError:
                    print("%s: %s" % (k, v))
            if not n.estados:
                print("no llegan los estados de go2_sensores / go2_guardian")
            return 0
        print(__doc__)
        return 1
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())

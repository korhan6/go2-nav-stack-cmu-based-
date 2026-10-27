"""go2_calibrar_fantasmas: calibra el filtro de fantasmas del L1 para ESTE perro y ESTE lugar.

  'go2v2 calibrar-fantasmas' (con 'go2v2 lanzar' corriendo), o 'go2v2 lanzar calibrar_fantasmas:=true':
    1. levanta al perro
    2. quieto unos segundos y despues UNA VUELTA en el sitio (~10 s), grabando el lidar
       (necesita ~1 m libre alrededor; lo mejor es un lugar con paredes y muebles cerca)
    3. separa lo REAL (lo que aparece en muchas nubes) del RUIDO (puntos sueltos que aparecen una vez)
       y elige min_puntos / ventana del filtro: el que mas ruido saca guardando >= 97 % de lo real
    4. lo guarda en ~/.go2_v2/fantasmas.yaml y go2_fantasmas lo usa al toque (y en cada 'go2v2 lanzar')

El giro va como el teclado ('go2v2 manejar'): modo manual de CMU, por el guardian (suavizado). CIRCULO del
PS4 (pausa) lo frena en cualquier momento.
"""
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, PointCloud2
from std_srvs.srv import Trigger

from .filtros import calibrar_fantasmas, xyz_de_nube

ARCHIVO = "~/.go2_v2/fantasmas.yaml"


def joy_giro(yaw):
    """Como 'go2v2 manejar': R2 (manual de CMU) y el eje de giro. yaw = 0 y R2 suelto: quieto."""
    j = Joy()
    j.header.frame_id = "teclado"
    j.axes = [float(yaw), 0.0, 1.0, 0.0, 0.0, -1.0 if yaw else 1.0, 0.0, 0.0]
    j.buttons = [0] * 11
    return j


class Calibrar(Node):
    def __init__(self):
        super().__init__("go2_calibrar_fantasmas")
        P = self.declare_parameter
        # fraccion del giro maximo de CMU (60 grados/s). 26/10: con 0.35 (21 grados/s) el Go2 casi no giraba
        # (como caminar a menos de 0.2 m/s: marca el paso en el sitio). 0.75 = 45 grados/s, una vuelta en ~8 s
        self.giro = float(P("giro", 0.75).value)
        self.vueltas = float(P("vueltas", 1.0).value)
        self.quieto_s = float(P("quieto_s", 3.0).value)
        self.max_s = float(P("max_s", 40.0).value)
        self.t_aviso = 0.0
        self.levantar = bool(P("levantar", True).value)
        self.archivo = os.path.expanduser(str(P("archivo", ARCHIVO).value))
        self.pub_joy = self.create_publisher(Joy, "/joy", 10)
        self.create_subscription(PointCloud2, "/registered_scan", self.cb_nube, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, qos_profile_sensor_data, raw=True)
        self.cli_levantar = self.create_client(Trigger, "/go2/levantar")
        self.cli_recargar = self.create_client(Trigger, "/go2/fantasmas_recargar")
        self.odom_raw = None
        self.pose = None                  # (x, y, z, yaw) del lidar
        self.yaw_prev = None
        self.girado = 0.0
        self.nubes, self.centros = [], []
        self.n_nubes = 0
        self.grabando = False
        self.estado = "esperando"
        self.t_estado = time.monotonic()
        self.n_parar = 0
        self.resultado = None
        self.terminado = False
        self.create_timer(0.1, self.paso)
        self.create_timer(0.05, self.mandar_giro)     # la orden de giro a 20 Hz (como el teclado, sin huecos)
        self.get_logger().warn("CALIBRAR FANTASMAS: el perro se va a parar y dar UNA vuelta en el sitio. "
                               "Deja ~1 m libre alrededor. CIRCULO del PS4 = pausa")

    def cb_odom(self, m):
        self.odom_raw = m

    def leer_pose(self):
        m, self.odom_raw = self.odom_raw, None
        if m is None:
            return
        if isinstance(m, (bytes, bytearray)):
            from rclpy.serialization import deserialize_message
            m = deserialize_message(m, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, p.z, yaw)
        if self.estado == "girando" and self.yaw_prev is not None:
            self.girado += abs(math.atan2(math.sin(yaw - self.yaw_prev), math.cos(yaw - self.yaw_prev)))
        self.yaw_prev = yaw

    def cb_nube(self, m):
        self.n_nubes += 1
        if not self.grabando or self.pose is None:
            return
        n = m.width * m.height
        campos = {f.name: f.offset for f in m.fields}
        if n == 0 or not all(k in campos for k in ("x", "y", "z")):
            return
        xyz, _ = xyz_de_nube(m.data, n, m.point_step, [campos[c] for c in ("x", "y", "z")])
        self.nubes.append((time.monotonic(), np.array(xyz, np.float32)))
        self.centros.append(self.pose[:3])

    def mandar_giro(self):
        if self.estado == "girando":
            self.pub_joy.publish(joy_giro(self.giro))

    def cambiar(self, estado):
        self.estado = estado
        self.t_estado = time.monotonic()

    def paso(self):
        self.leer_pose()
        dt = time.monotonic() - self.t_estado
        if self.estado == "esperando":
            if self.n_nubes > 5 and self.pose is not None:
                if self.levantar and self.cli_levantar.service_is_ready():
                    self.cli_levantar.call_async(Trigger.Request())
                    self.get_logger().info("levantando...")
                self.cambiar("levantando")
            elif dt > 60.0:
                self.get_logger().error("no llegan el lidar o la pose en 60 s: esta corriendo 'go2v2 lanzar'?")
                self.terminado = True
        elif self.estado == "levantando":
            if dt > (4.5 if self.levantar else 0.5):
                self.grabando = True
                self.get_logger().info("grabando quieto %.0f s..." % self.quieto_s)
                self.cambiar("quieto")
        elif self.estado == "quieto":
            if dt > self.quieto_s:
                self.girado = 0.0
                self.t_aviso = 0.0
                self.get_logger().info("girando %.1f vuelta(s) a %.0f grados/s..." % (self.vueltas, 60 * self.giro))
                self.cambiar("girando")
        elif self.estado == "girando":
            if dt - self.t_aviso >= 2.0:
                self.t_aviso = dt
                self.get_logger().info("girado %.0f de %.0f grados (%.0f grados/s)"
                                       % (math.degrees(self.girado), 360 * self.vueltas,
                                          math.degrees(self.girado) / max(dt, 0.1)))
            if self.girado >= self.vueltas * 2 * math.pi or dt > self.max_s:
                if self.girado < self.vueltas * 2 * math.pi:
                    self.get_logger().warn("no completo la vuelta en %.0f s (giro %.0f grados): calibro con lo que "
                                           "hay. Si gira muy lento: go2v2 calibrar-fantasmas 1.0"
                                           % (self.max_s, math.degrees(self.girado)))
                self.cambiar("parando")
        elif self.estado == "parando":
            self.pub_joy.publish(joy_giro(0.0))
            self.n_parar += 1
            if self.n_parar >= 5:
                self.grabando = False
                self.cambiar("calculando")
        elif self.estado == "calculando":
            self.calcular()
            self.terminado = True
            self.cambiar("fin")

    def calcular(self):
        n = len(self.nubes)
        self.get_logger().info("calculando con %d nubes (giro %.0f grados)..." % (n, math.degrees(self.girado)))
        if n:
            t0 = self.nubes[0][0]
            nubes = [(t - t0, xyz) for t, xyz in self.nubes]
        r = calibrar_fantasmas(nubes, self.centros) if n else None
        if r is None:
            self.get_logger().error("no alcanzan los datos (%d nubes): no cambio nada" % n)
            return
        self.resultado = r
        import yaml
        os.makedirs(os.path.dirname(self.archivo), exist_ok=True)
        with open(self.archivo, "w") as f:
            f.write("# go2v2 calibrar-fantasmas (%s). go2_fantasmas lo lee al arrancar\n"
                    % time.strftime("%Y-%m-%d %H:%M"))
            yaml.safe_dump(r, f, sort_keys=False)
        if self.cli_recargar.service_is_ready():
            self.cli_recargar.call_async(Trigger.Request())
        self.get_logger().warn("LISTO: min_puntos %d, ventana %.2f s -> guarda %.1f %% de lo real, saca %.1f %% del "
                               "ruido. Guardado en %s (lo usa go2_fantasmas desde ya y en cada lanzar)%s"
                               % (r["min_puntos"], r["ventana"], 100 * r["guarda_real"], 100 * r["quita_fantasmas"],
                                  self.archivo, (". OJO: " + r["aviso"]) if r.get("aviso") else ""))


def main():
    rclpy.init()
    n = Calibrar()
    try:
        while rclpy.ok() and not n.terminado:
            rclpy.spin_once(n, timeout_sec=0.1)
        for _ in range(5):                       # que la respuesta del recargar alcance a salir
            rclpy.spin_once(n, timeout_sec=0.1)
    except KeyboardInterrupt:
        for _ in range(5):
            n.pub_joy.publish(joy_giro(0.0))
            time.sleep(0.05)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

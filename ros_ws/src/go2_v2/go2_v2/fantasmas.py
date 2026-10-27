"""go2_fantasmas: saca los puntos fantasma del L1 antes del analisis de terreno de CMU.

  /registered_scan (Point-LIO, marco del mapa) -> /registered_scan_limpia (terrainAnalysis y
  terrainAnalysisExt la leen en vez de la cruda; Point-LIO, FAR y el mapa siguen con la cruda)

Regla (go2_v2/filtros.py): un punto cerca del perro se queda solo si en su vecindario de 30x30x30 cm
hubo al menos 'min_puntos' puntos en las ultimas 'ventana' s. Lo real se repite nube tras nube en el
mismo lugar; el ruido no. El piso nunca se toca (ver filtros.py). activo:=false -> pasa todo igual.
Estado (JSON 1 Hz) en /go2/fantasmas_estado: puntos que entran, fantasmas sacados, ms.
Calibracion: si existe ~/.go2_v2/fantasmas.yaml ('go2v2 calibrar-fantasmas') usa su min_puntos y ventana
en vez de los del launch. /go2/fantasmas_recargar la vuelve a leer sin relanzar.
"""
import os
import array
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from .filtros import FiltroFantasmas, xyz_de_nube

FLOAT32 = 7


class Fantasmas(Node):
    def __init__(self):
        super().__init__("go2_fantasmas")
        P = self.declare_parameter
        self.activo = bool(P("activo", True).value)
        self.f = FiltroFantasmas(tam=float(P("voxel", 0.1).value), min_puntos=int(P("min_puntos", 5).value),
                                 ventana=float(P("ventana", 0.5).value), radio=float(P("radio", 4.0).value),
                                 z_min_rel=float(P("z_min_rel", -0.2).value))
        self.archivo = os.path.expanduser(str(P("calibracion", "~/.go2_v2/fantasmas.yaml").value))
        self.usar_calibracion = bool(P("usar_calibracion", True).value)
        self.calibrado = None
        self.cargar_calibracion()
        from std_srvs.srv import Trigger
        self.create_service(Trigger, "/go2/fantasmas_recargar", self.srv_recargar)
        self.centro = None
        self.odom = None
        self.pub = self.create_publisher(PointCloud2, "/registered_scan_limpia", 5)
        self.pub_est = self.create_publisher(String, "/go2/fantasmas_estado", 5)
        # Point-LIO publica la pose a MILES por segundo (una por grupo de puntos): aca solo se guarda el
        # ultimo mensaje SIN deserializar (raw) y se lee una vez por nube. Deserializar cada uno en Python
        # se comia la CPU y atrasaba la nube limpia (y con ella el mapa de terreno).
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, qos_profile_sensor_data, raw=True)
        self.create_subscription(PointCloud2, "/registered_scan", self.cb_nube, 5)
        self.est = {"nubes": 0, "puntos": 0, "fantasmas": 0, "ms": 0.0}
        self.create_timer(1.0, self.estado)
        self.get_logger().info("go2_fantasmas %s: /registered_scan -> /registered_scan_limpia (min %d puntos "
                               "en %.1f s, radio %.1f m)" % ("ACTIVO" if self.activo else "apagado (pasa todo)",
                                                           self.f.min_puntos, self.f.ventana, self.f.radio))

    def cargar_calibracion(self):
        if not self.usar_calibracion:
            return False
        try:
            import yaml
            with open(self.archivo) as f:
                d = yaml.safe_load(f) or {}
            self.f.min_puntos = int(d["min_puntos"])
            self.f.ventana = float(d["ventana"])
            self.calibrado = {k: d.get(k) for k in ("min_puntos", "ventana", "guarda_real", "quita_fantasmas")}
            self.get_logger().info("fantasmas CALIBRADO (%s): min_puntos %d, ventana %.2f s"
                                   % (self.archivo, self.f.min_puntos, self.f.ventana))
            return True
        except FileNotFoundError:
            self.get_logger().info("fantasmas sin calibrar (min_puntos %d, ventana %.2f s): 'go2v2 calibrar-fantasmas'"
                                   % (self.f.min_puntos, self.f.ventana))
        except Exception as e:      # archivo roto: sigue con los valores del launch
            self.get_logger().warn("no pude leer %s (%s): sigo con min_puntos %d" % (self.archivo, e, self.f.min_puntos))
        return False

    def srv_recargar(self, _req, res):
        res.success = self.cargar_calibracion()
        res.message = ("min_puntos %d, ventana %.2f s" % (self.f.min_puntos, self.f.ventana) if res.success
                       else "sin calibracion en %s" % self.archivo)
        return res

    def cb_odom(self, m):
        self.odom = m

    def leer_centro(self):
        m, self.odom = self.odom, None
        if m is None:
            return self.centro
        if isinstance(m, (bytes, bytearray)):
            from rclpy.serialization import deserialize_message
            m = deserialize_message(m, Odometry)
        p = m.pose.pose.position
        self.centro = (p.x, p.y, p.z)
        return self.centro

    def cb_nube(self, m):
        n = m.width * m.height
        if not self.activo or self.leer_centro() is None or n == 0:
            self.pub.publish(m)
            return
        campos = {f.name: f for f in m.fields}
        if not all(k in campos and campos[k].datatype == FLOAT32 for k in ("x", "y", "z")):
            self.pub.publish(m)
            return
        t0 = time.perf_counter()
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        xyz, crudo = xyz_de_nube(m.data, n, m.point_step, [campos[c].offset for c in ("x", "y", "z")])
        ok = self.f.filtrar(st, xyz, self.centro)
        k = int(ok.sum())
        out = PointCloud2()
        out.header = m.header
        out.height, out.width = 1, k
        out.fields = m.fields
        out.is_bigendian = m.is_bigendian
        out.point_step = m.point_step
        out.row_step = m.point_step * k
        out.is_dense = m.is_dense
        buf = array.array("B")
        buf.frombytes(crudo[ok].tobytes())
        out.data = buf
        self.pub.publish(out)
        e = self.est
        e["nubes"] += 1
        e["puntos"] += n
        e["fantasmas"] += n - k
        e["ms"] = max(e["ms"], (time.perf_counter() - t0) * 1000.0)

    def estado(self):
        e = dict(self.est)
        e["ms"] = round(e["ms"], 2)
        e["activo"] = self.activo
        e["min_puntos"] = self.f.min_puntos
        e["ventana"] = self.f.ventana
        e["calibrado"] = self.calibrado is not None
        self.pub_est.publish(String(data=json.dumps(e)))
        self.est = {"nubes": 0, "puntos": 0, "fantasmas": 0, "ms": 0.0}


def main():
    rclpy.init()
    n = Fantasmas()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

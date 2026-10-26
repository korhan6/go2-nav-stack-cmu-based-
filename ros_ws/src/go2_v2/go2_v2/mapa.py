"""go2_mapa: arma el mapa 3D con lo que ve el lidar mientras el perro camina (manejado con el
teclado, el PS4 o navegando solo) y lo guarda.

  /registered_scan (Point-LIO, marco del mapa) -> voxeles de 5 cm
  /go2/mapa (PointCloud2, cada 3 s, version de 10 cm para que RViz no sufra)
  servicios: /go2/guardar_mapa -> ~/.go2_v2/mapas/mapa_FECHA.pcd (5 cm; CloudCompare, Open3D, PCL)
             /go2/borrar_mapa
  Ojo: el mapa se arma en el marco de Point-LIO, que empieza donde se prendio el perro. Por ahora es
  para verlo y guardarlo; la navegacion no lo vuelve a cargar (relocalizacion: paso siguiente).
"""
import array
import datetime
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger

from .filtros import AcumuladorMapa, escribir_pcd, xyz_de_nube

FLOAT32 = 7


class Mapa(Node):
    def __init__(self):
        super().__init__("go2_mapa")
        P = self.declare_parameter
        self.m = AcumuladorMapa(tam=float(P("voxel", 0.05).value))
        self.tam_rviz = float(P("voxel_rviz", 0.1).value)
        self.carpeta = os.path.expanduser(P("carpeta", "~/.go2_v2/mapas").value)
        self.marco = "camera_init"
        self.pub = self.create_publisher(PointCloud2, "/go2/mapa", 1)
        # best effort: si este nodo se atrasa, no frena a Point-LIO
        self.create_subscription(PointCloud2, "/registered_scan", self.cb_nube, qos_profile_sensor_data)
        self.create_service(Trigger, "/go2/guardar_mapa", self.srv_guardar)
        self.create_service(Trigger, "/go2/borrar_mapa", self.srv_borrar)
        self.create_timer(1.0, self.m.consolidar)
        self.create_timer(float(P("publicar_cada", 3.0).value), self.publicar)
        self.get_logger().info("go2_mapa: armando el mapa (voxel %.2f m). Guardar: 'go2v2 mapa' o G en el teclado"
                               % self.m.tam)

    def cb_nube(self, m):
        n = m.width * m.height
        campos = {f.name: f for f in m.fields}
        if n == 0 or not all(k in campos and campos[k].datatype == FLOAT32 for k in ("x", "y", "z")):
            return
        self.marco = m.header.frame_id or self.marco
        xyz, _ = xyz_de_nube(m.data, n, m.point_step, [campos[c].offset for c in ("x", "y", "z")])
        self.m.agregar(xyz)

    def publicar(self):
        xyz = self.m.puntos(tam_salida=self.tam_rviz)
        if len(xyz) == 0:
            return
        out = PointCloud2()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.marco
        out.height, out.width = 1, len(xyz)
        out.fields = [PointField(name=c, offset=4 * i, datatype=FLOAT32, count=1) for i, c in enumerate("xyz")]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * len(xyz)
        out.is_dense = True
        buf = array.array("B")
        buf.frombytes(xyz.tobytes())
        out.data = buf
        self.pub.publish(out)

    def srv_guardar(self, _req, res):
        xyz = self.m.puntos()
        if len(xyz) == 0:
            res.success, res.message = False, "el mapa esta vacio (llega /registered_scan?)"
            return res
        os.makedirs(self.carpeta, exist_ok=True)
        ruta = os.path.join(self.carpeta, "mapa_%s.pcd" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        n = escribir_pcd(ruta, xyz)
        res.success, res.message = True, "%d puntos -> %s" % (n, ruta)
        self.get_logger().info("mapa guardado: " + res.message)
        return res

    def srv_borrar(self, _req, res):
        self.m.borrar()
        res.success, res.message = True, "mapa borrado"
        return res


def main():
    rclpy.init()
    n = Mapa()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

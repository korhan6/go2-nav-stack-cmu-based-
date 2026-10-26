"""go2_metas: decide COMO ir a cada meta. Es el unico que publica /way_point (lo que sigue el
planificador local de CMU).

Por que existe (grabacion del 26/10): con metas cortas (1-2 m) dentro de una pieza, FAR planner salta
de un vertice a otro de su grafo (way_points a la izquierda, a la derecha, atras...). El perro
persigue cada uno: gira a +80 grados/s, despues a -80, y se queda trabado a 0.6 m de la meta porque
FAR nunca le da la meta final. FAR esta hecho para rutas largas por lugares desconocidos.

Regla:
  * meta a menos de 'directo_max' m y con la linea recta libre en el mapa de terreno
      -> DIRECTO: /way_point = la meta (el planificador local esquiva solo, a 100 Hz)
  * si no -> FAR: la meta va a FAR (/far_goal) y sus way_points se pasan a /way_point
  * yendo con FAR, en cuanto la meta queda a menos de 'directo_max' con la linea libre -> DIRECTO
    (llega a la meta exacta, no al ultimo vertice de FAR)
  * yendo DIRECTO, si la linea se tapa y falta mas de 1 m -> vuelve a FAR

  /goal_point (RViz Goalpoint, 'go2v2 ir') -> /far_goal (a FAR) o /way_point (directo)
  /far_way_point (salida de FAR, remapeada) -> /way_point (solo en modo FAR)
  /go2/metas_estado (JSON 2 Hz): modo, distancia, linea libre
"""
import json
import math
import os

import numpy as np

ALTURA_OBSTACULO = 0.3      # m: igual que obstacleHeightThre del planificador local


def linea_libre(origen, meta, obs_xy, ancho=0.25, ignorar_cerca=0.35):
    """True si no hay obstaculos (obs_xy: (n, 2) en el mapa) a menos de 'ancho' m del segmento
    origen -> meta. Se ignora lo que esta pegado al perro (sus patas, ruido del L1) y pegado a la meta
    (si la meta esta contra una pared, igual se puede llegar cerca)."""
    if len(obs_xy) == 0:
        return True
    a = np.asarray(origen[:2], float)
    b = np.asarray(meta[:2], float)
    ab = b - a
    largo = float(np.hypot(*ab))
    if largo < 1e-6:
        return True
    p = np.asarray(obs_xy, float)
    s = np.clip(((p - a) @ ab) / (largo * largo), 0.0, 1.0)
    cerca = p - (a + s[:, None] * ab)
    d = np.hypot(cerca[:, 0], cerca[:, 1])
    da = np.hypot(*(p - a).T)
    db = np.hypot(*(p - b).T)
    bloquea = (d < ancho) & (da > ignorar_cerca) & (db > ignorar_cerca)
    return int(bloquea.sum()) < 2           # como pointPerPathThre de CMU: 2 puntos bloquean


class Metas(object):
    """Logica sin ROS. Devuelve listas de ('far', (x,y,z)) / ('way', (x,y,z)) para publicar."""

    def __init__(self, directo_max=3.0, volver_far=1.0, espera_far=4.0):
        self.directo_max = directo_max
        self.volver_far = volver_far
        # s sin un solo way_point de FAR = FAR no encuentra ruta (26/10: meta en la escalera, FAR callado
        # 30 s y el perro dando vueltas con un way_point viejo) -> el perro se queda quieto y avisa
        self.espera_far = espera_far
        self.meta = None
        self.modo = None            # 'directo' | 'far' | 'sin_ruta' | None
        self.t_far = None           # ultimo way_point de FAR (o cuando se paso a FAR)
        self.avisos = []

    def nueva_meta(self, meta, pos, obs_xy, t=0.0):
        self.meta = tuple(meta)
        if self._puede_directo(pos, obs_xy):
            self.modo = "directo"
            return [("way", self.meta)]
        self.modo = "far"
        self.t_far = t
        return [("far", self.meta)]

    def _puede_directo(self, pos, obs_xy):
        if pos is None:
            return False
        dist = math.hypot(self.meta[0] - pos[0], self.meta[1] - pos[1])
        return dist <= self.directo_max and linea_libre(pos, self.meta, obs_xy)

    def way_de_far(self, w, t=0.0):
        if self.modo == "sin_ruta":
            return []
        self.t_far = t
        return [("way", tuple(w))] if self.modo == "far" else []

    def paso(self, pos, obs_xy, t=0.0):
        """Llamar seguido (5 Hz). Cambia de modo si hace falta y repite la meta en modo directo."""
        if self.meta is None or pos is None or self.modo == "sin_ruta":
            return []
        if self.modo == "far" and self._puede_directo(pos, obs_xy):
            self.modo = "directo"
        elif self.modo == "far" and self.t_far is not None and t - self.t_far > self.espera_far:
            self.modo = "sin_ruta"
            self.avisos.append("FAR no encuentra ruta a (%.2f, %.2f) hace %.0f s: me quedo quieto. La meta "
                               "esta en un lugar al que no se llega (escalera, otro piso, detras de algo "
                               "cerrado) o todavia no conoce el camino: acercalo o pon la meta mas cerca"
                               % (self.meta[0], self.meta[1], t - self.t_far))
            return [("way", (pos[0], pos[1], pos[2]))]
        elif self.modo == "directo":
            dist = math.hypot(self.meta[0] - pos[0], self.meta[1] - pos[1])
            if dist > self.volver_far and not linea_libre(pos, self.meta, obs_xy):
                self.modo = "far"
                self.t_far = t
                return [("far", self.meta)]
        if self.modo == "directo":
            return [("way", self.meta)]
        return []


class MetasPropio(object):
    """Planificador propio (go2_v2/ruta.py, A*): replanifica en cada paso sobre la grilla y le da al
    planificador local de CMU un way_point 'adelante' m mas alla sobre la ruta. Si la meta no se alcanza
    (escalera, contra algo), va al punto alcanzable mas cerca y avisa una vez."""

    def __init__(self, grilla, adelante=1.2, espera=4.0, llegada=0.35):
        self.g = grilla
        self.adelante = adelante
        self.espera = espera            # s sin ruta posible -> quieto y aviso
        self.llegada = llegada
        self.meta = None
        self.modo = None                # 'propio' | 'llegando' | 'sin_ruta' | None
        self.camino = None
        self.t_ok = None
        self.parcial_avisado = False
        self.avisos = []
        self.way = None             # el way_point que tiene el planificador local

    def nueva_meta(self, meta, t=0.0):
        self.meta = tuple(meta)
        self.modo = "propio"
        self.camino = None
        self.t_ok = t
        self.parcial_avisado = False
        self.way = None

    def paso(self, pos, t=0.0):
        if self.meta is None or pos is None or self.modo == "sin_ruta":
            return []
        dist = math.hypot(self.meta[0] - pos[0], self.meta[1] - pos[1])
        if self.modo == "llego":
            if dist < self.llegada + 0.4:           # quieto en la meta: no replanificar ni mover nada
                return []
            self.modo = "propio"                    # lo movieron lejos: vuelve
            self.way = None
        if dist < self.llegada:
            self.modo = "llego"
            self.camino = None
            self.way = self.meta
            return [("way", self.meta)]
        c = self.g.planificar(pos[:2], self.meta[:2], lo_mas_cerca=True)
        if c is None or len(c) < 2:
            if c is not None:                      # ya esta en el destino
                return [("way", (pos[0], pos[1], pos[2]))]
            if t - self.t_ok > self.espera:
                self.modo = "sin_ruta"
                self.avisos.append("no hay ruta a (%.2f, %.2f) desde donde esta el perro (encerrado o "
                                   "el mapa todavia no lo conoce): me quedo quieto" % self.meta[:2])
                return [("way", (pos[0], pos[1], pos[2]))]
            return []
        self.t_ok = t
        self.camino = c
        fin = c[-1]
        falta = math.hypot(fin[0] - self.meta[0], fin[1] - self.meta[1])
        if falta > 0.5 and not self.parcial_avisado:
            self.parcial_avisado = True
            self.avisos.append("la meta (%.2f, %.2f) no se alcanza (escalera, pared, algo cerrado): voy lo mas "
                               "cerca que se puede, a %.1f m de ella" % (self.meta[0], self.meta[1], falta))
        from .ruta import punto_adelante
        w = punto_adelante(c, self.adelante)
        # no mover el way_point por cada replanificacion (la ruta tiembla 10 cm y el perro cambiaba de
        # rumbo a cada rato): solo si cambio de verdad o si ya se esta acercando al anterior
        if self.way is not None and w != c[-1]:
            cambio = math.hypot(w[0] - self.way[0], w[1] - self.way[1])
            cerca = math.hypot(pos[0] - self.way[0], pos[1] - self.way[1])
            if cambio < 0.4 and cerca > 0.6:
                return []
        self.way = (w[0], w[1], self.meta[2])
        return [("way", self.way)]


def main():
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String

    from .filtros import xyz_de_nube

    class Nodo(Node):
        def __init__(self):
            super().__init__("go2_metas")
            P = self.declare_parameter
            self.sensor_x = float(P("sensorOffsetX", 0.3).value)
            self.m = Metas(directo_max=float(P("directo_max", 3.0).value))
            self.altura = float(P("altura_obstaculo", ALTURA_OBSTACULO).value)
            # planificador: 'propio' (A* de go2_v2/ruta.py, por defecto) o 'far' (FAR de CMU + directo)
            self.propio = str(P("planificador", "propio").value) != "far"
            from .ruta import Grilla
            self.grilla = Grilla(res=0.1, lado=float(P("mapa_lado", 60.0).value),
                                 altura=float(P("ruta_altura", 0.3).value),
                                 radio=float(P("ruta_radio", 0.25).value))
            self.mp = MetasPropio(self.grilla, adelante=float(P("adelante", 1.2).value))
            from nav_msgs.msg import Path
            self.Path = Path
            self.pub_ruta = self.create_publisher(Path, "/go2/ruta", 2)
            # el mapa de terreno de CMU, SIN los fantasmas que el perro ya atraveso (pisadas): lo usan el
            # planificador local y el guardian (launch: remap /terrain_map -> /terrain_map_go2)
            self.pub_terreno = self.create_publisher(PointCloud2, "/terrain_map_go2", 2)
            from nav_msgs.msg import OccupancyGrid
            self.OccupancyGrid = OccupancyGrid
            self.pub_grilla = self.create_publisher(OccupancyGrid, "/go2/grilla", 1)
            self.carpeta = os.path.expanduser(str(P("carpeta", "~/.go2_v2/mapas").value))
            self.yaw = 0.0
            self.nube = None
            self.n_borrados = 0
            cargar = str(P("cargar_mapa", "").value)
            if cargar:
                ruta = os.path.join(self.carpeta, "grilla_ultima.npz") if cargar == "ultimo" else os.path.expanduser(cargar)
                try:
                    self.grilla.cargar(ruta)
                    self.get_logger().warn("mapa cargado: %s. El perro tiene que haber arrancado DONDE y MIRANDO "
                                           "hacia donde arranco cuando se guardo" % ruta)
                except (OSError, ValueError, KeyError) as e:
                    self.get_logger().error("no pude cargar el mapa %s: %s (sigo con el mapa vacio)" % (ruta, e))
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "/go2/guardar_grilla", self.srv_guardar)
            self.create_service(Trigger, "/go2/olvidar_pisadas", self.srv_olvidar)
            self.create_timer(1.0, self.publicar_grilla)
            self.odom = None
            self.terreno = None
            self.pub_way = self.create_publisher(PointStamped, "/way_point", 5)
            self.pub_far = self.create_publisher(PointStamped, "/far_goal", 5)
            self.pub_est = self.create_publisher(String, "/go2/metas_estado", 5)
            # la pose sale a miles por segundo: se guarda cruda y se lee cuando hace falta
            self.create_subscription(Odometry, "/state_estimation", self.cb_odom, qos_profile_sensor_data, raw=True)
            self.create_subscription(PointCloud2, "/terrain_map", self.cb_terreno, 2)
            self.create_subscription(PointStamped, "/goal_point", self.cb_meta, 5)
            self.create_subscription(PointStamped, "/far_way_point", self.cb_far, 5)
            # SLAM reiniciado (go2v2 slam-reset): la meta vieja esta en el mapa viejo -> se olvida
            from std_msgs.msg import Empty
            self.create_subscription(Empty, "/reset_visibility_graph", self.cb_reset, 5)
            self.create_timer(0.2, self.paso)
            self.create_timer(0.5, self.estado)
            self.pos = None
            if self.propio:
                self.get_logger().info("go2_metas: planificador PROPIO (A*, ruta en /go2/ruta); FAR no se usa")
            else:
                self.get_logger().info("go2_metas: metas a menos de %.1f m con la linea libre van DIRECTO; "
                                       "el resto, con FAR" % self.m.directo_max)

        def t(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def cb_odom(self, raw):
            self.odom = raw

        def cb_terreno(self, m):
            """Saca los fantasmas pisados y lo pasa al planificador local (en el momento: es su entrada)."""
            n = m.width * m.height
            campos = {f.name: f.offset for f in m.fields}
            if n == 0 or "intensity" not in campos or self.grilla.pisadas is None:
                self.pub_terreno.publish(m)
                return
            xyz, crudo = xyz_de_nube(m.data, n, m.point_step, [campos[c] for c in ("x", "y", "z")])
            o = campos["intensity"]
            h = np.ascontiguousarray(crudo[:, o:o + 4]).view(np.float32)[:, 0]
            p = np.c_[xyz, h]
            ok = self.grilla.filtrar_pisadas(p)
            if ok.all():
                self.pub_terreno.publish(m)
            else:
                self.n_borrados += int((~ok).sum())
                import array
                out = PointCloud2()
                out.header = m.header
                out.height, out.width = 1, int(ok.sum())
                out.fields = m.fields
                out.is_bigendian = m.is_bigendian
                out.point_step = m.point_step
                out.row_step = m.point_step * out.width
                out.is_dense = m.is_dense
                buf = array.array("B")
                buf.frombytes(crudo[ok].tobytes())
                out.data = buf
                self.pub_terreno.publish(out)
            self.nube = p[ok & (p[:, 3] < 1.49)]       # sin las 'paredes sin datos' pegadas al perro

        def srv_guardar(self, _req, res):
            os.makedirs(self.carpeta, exist_ok=True)
            import datetime
            nombre = os.path.join(self.carpeta, "grilla_%s.npz" % datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.grilla.guardar(nombre)
            import shutil
            shutil.copy(nombre, os.path.join(self.carpeta, "grilla_ultima.npz"))
            res.success, res.message = True, "mapa de rutas guardado: %s (y grilla_ultima.npz)" % nombre
            self.get_logger().info(res.message)
            return res

        def srv_olvidar(self, _req, res):
            self.grilla.olvidar_pisadas()
            res.success, res.message = True, "pisadas olvidadas"
            return res

        def publicar_grilla(self):
            """La grilla del planificador en RViz (16x16 m alrededor del perro): negro = obstaculo."""
            if self.pos is None:
                return
            g = self.grilla
            ci, cj = g.celda(self.pos[0], self.pos[1])
            r = int(8.0 / g.res)
            i0, j0 = max(0, ci - r), max(0, cj - r)
            i1, j1 = min(g.n, ci + r), min(g.n, cj + r)
            sub = g.c[i0:i1, j0:j1]
            val = np.where(sub == 1, 100, np.where(sub == 0, 0, -1)).astype(np.int8)
            m = self.OccupancyGrid()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "map"
            m.info.resolution = float(g.res)
            m.info.width, m.info.height = int(i1 - i0), int(j1 - j0)
            x0, y0 = g.centro(i0, j0)
            m.info.origin.position.x = float(x0 - g.res / 2)
            m.info.origin.position.y = float(y0 - g.res / 2)
            m.info.origin.position.z = float(self.pos[2]) - 0.35
            m.info.origin.orientation.w = 1.0
            m.data = val.T.reshape(-1).tolist()        # OccupancyGrid: fila = y, columna = x
            self.pub_grilla.publish(m)

        def leer_pos(self):
            if self.odom is not None:
                o = deserialize_message(self.odom, Odometry)
                self.odom = None
                p, q = o.pose.pose.position, o.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                # centro del cuerpo (el L1 va adelante), como lo calcula CMU
                self.pos = (p.x - math.cos(yaw) * self.sensor_x, p.y - math.sin(yaw) * self.sensor_x, p.z)
                self.yaw = yaw
            return self.pos

        def nube_terreno(self):
            p, self.nube = self.nube, None
            return p

        def actualizar_grilla(self, pos):
            p = self.nube_terreno()
            if p is None or pos is None:
                return
            cerca = p[(np.hypot(p[:, 0] - pos[0], p[:, 1] - pos[1]) < 1.2) & (p[:, 3] < 0.1)]
            piso = float(np.median(cerca[:, 2])) if len(cerca) > 20 else pos[2] - 0.35
            self.grilla.actualizar(p, piso)

        def obstaculos(self):
            p = self.nube
            if p is None:
                return np.zeros((0, 2))
            return p[p[:, 3] > self.altura, :2]

        def publicar(self, salida):
            for tipo, p in salida:
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.point.x, msg.point.y, msg.point.z = float(p[0]), float(p[1]), float(p[2])
                (self.pub_way if tipo == "way" else self.pub_far).publish(msg)

        def cb_meta(self, m):
            meta = (m.point.x, m.point.y, m.point.z)
            if self.propio:
                self.mp.nueva_meta(meta, self.t())
                self.get_logger().info("meta (%.2f, %.2f): planificador propio" % (meta[0], meta[1]))
                return
            self.publicar(self.m.nueva_meta(meta, self.leer_pos(), self.obstaculos(), self.t()))
            self.get_logger().info("meta (%.2f, %.2f): voy %s" % (meta[0], meta[1], self.m.modo.upper()))

        def cb_reset(self, _m):
            self.m.meta, self.m.modo = None, None
            self.mp.meta, self.mp.modo, self.mp.camino = None, None, None
            self.grilla.c[:] = -1
            self.grilla.olvidar_pisadas()
            self.pos = None

        def cb_far(self, m):
            self.publicar(self.m.way_de_far((m.point.x, m.point.y, m.point.z), self.t()))

        def paso(self):
            pos = self.leer_pos()
            if pos is not None:
                # por donde paso el cuerpo no hay nada solido (manejado con el PS4/teclado o solo)
                self.grilla.pisar(pos[0], pos[1], self.yaw)
            if self.propio:
                self.actualizar_grilla(pos)
                self.publicar(self.mp.paso(pos, self.t()))
                for a in self.mp.avisos:
                    self.get_logger().warn(a)
                self.mp.avisos = []
                if self.mp.camino and self.mp.modo == "propio":
                    from geometry_msgs.msg import PoseStamped
                    r = self.Path()
                    r.header.stamp = self.get_clock().now().to_msg()
                    r.header.frame_id = "map"
                    for x, y in self.mp.camino:
                        ps = PoseStamped()
                        ps.header = r.header
                        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
                        ps.pose.position.z = float(pos[2]) - 0.3 if pos else 0.0
                        r.poses.append(ps)
                    self.pub_ruta.publish(r)
                return
            antes = self.m.modo
            pos = self.leer_pos()
            obs = self.obstaculos() if self.m.meta is not None else np.zeros((0, 2))
            self.publicar(self.m.paso(pos, obs, self.t()))
            for a in self.m.avisos:
                self.get_logger().warn(a)
            self.m.avisos = []
            if self.m.modo != antes and antes is not None:
                self.get_logger().info("meta: cambio a %s" % self.m.modo.upper())

        def estado(self):
            m = self.mp if self.propio else self.m
            d = {"planificador": "propio" if self.propio else "far", "modo": m.modo, "meta": m.meta}
            if m.meta is not None and self.pos is not None:
                d["distancia"] = round(math.hypot(m.meta[0] - self.pos[0], m.meta[1] - self.pos[1]), 2)
            d["fantasmas_pisados_borrados"] = self.n_borrados
            self.n_borrados = 0
            if self.propio and self.mp.camino:
                from .ruta import largo
                d["ruta_m"] = round(largo(self.mp.camino), 2)
            self.pub_est.publish(String(data=json.dumps(d)))

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
    main()

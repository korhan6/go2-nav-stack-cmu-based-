"""go2_guardian: el unico nodo que le habla al perro (ver guardian_logica.py).

  /cmd_vel (geometry_msgs/TwistStamped, pathFollower de CMU)  -> /api/sport/request (Move/StopMove)
  /joy (PS4, solo el control fisico) -> SHARE+CRUZ acostar | SHARE+TRIANGULO levantar |
                                    SHARE+CUADRADO bloquear | SHARE+CIRCULO recuperar | CIRCULO pausa
                                    (L1 y R1 los usa CMU: borrar grafo de rutas / mapa de terreno)
  /terrain_map (solo la hora)    -> sin mapa de terreno > 1.5 s: quieto (la evasion no ve)
  servicios std_srvs/Trigger     -> /go2/acostar /go2/levantar /go2/bloquear /go2/desbloquear
                                    /go2/pausa /go2/recuperar
  /go2/guardian_estado (JSON 2 Hz)
Ademas apaga la evasion de obstaculos de Unitree (frena al perro y lo deja pisando en el sitio: la
evasion la hace el planificador de CMU) y manda StopMove al arrancar y al cerrarse.
"""
import itertools
import math
import json
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy, PointCloud2
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger

from unitree_api.msg import Request, Response

from .guardian_logica import API_STOPMOVE, Guardian

EVITA_SET, EVITA_GET = 1001, 1002          # servicio "obstacles_avoid" de Unitree
# /joy que NO son el control: los botones de RViz de CMU, 'go2v2 ir/parar' y 'go2v2 manejar'
JOY_AJENOS = ("goalpoint_tool", "waypoint_tool", "teleop_panel", "go2v2", "teclado")


class NodoGuardian(Node):
    def __init__(self):
        super().__init__("go2_guardian")
        P = self.declare_parameter
        self.g = Guardian(vx_max=float(P("vx_max", 1.0).value), vy_max=float(P("vy_max", 0.6).value),
                          wz_max=float(P("wz_max", 1.5).value), vigencia=float(P("vigencia", 0.25).value),
                          freewalk=bool(P("freewalk", True).value),
                          v_min_caminar=float(P("v_min_caminar", 0.0).value),
                          odom_max=float(P("odom_max_edad", 0.6).value),
                          terreno_max=float(P("terreno_max_edad", 1.5).value),
                          lidar_max=float(P("lidar_max_edad", 0.5).value),
                          giro_max=float(P("giro_max_s", 6.0).value),
                          salto_vel=float(P("salto_vel", 2.5).value),
                          salto_giro=float(P("salto_giro", 4.0).value),
                          vueltas_rad=float(P("vueltas_rad", 4.0).value),
                          vueltas_s=float(P("vueltas_s", 8.0).value),
                          sensor_x=float(P("sensorOffsetX", 0.3).value),
                          coherencia_m=float(P("coherencia_m", 0.05).value))
        self.apagar_evita = bool(P("apagar_evitacion_unitree", True).value)
        self.lock = threading.Lock()
        self.ids = itertools.count(int(time.time() * 1000) % 1_000_000_000)   # seguro entre hilos
        self.pub_req = self.create_publisher(Request, "/api/sport/request", 10)
        self.pub_evita = self.create_publisher(Request, "/api/obstacles_avoid/request", 10)
        self.pub_stop = self.create_publisher(Int8, "/stop", 5)
        self.pub_est = self.create_publisher(String, "/go2/guardian_estado", 5)
        grupo = MutuallyExclusiveCallbackGroup()
        self.create_subscription(TwistStamped, "/cmd_vel", self.cb_cmd, 10, callback_group=grupo)
        self.create_subscription(Joy, "/joy", self.cb_joy, 10, callback_group=grupo)
        self.create_subscription(Response, "/api/obstacles_avoid/response", self.cb_evita, 10, callback_group=grupo)
        # el mapa de terreno: solo importa que llegue (raw: sin deserializar la nube en Python)
        self.create_subscription(PointCloud2, "/terrain_map", self.cb_terreno, 2, callback_group=grupo, raw=True)
        # la nube que Point-LIO YA registro: si deja de llegar, la pose sale solo de la IMU (drift).
        # best effort: no frena a Point-LIO si este nodo se atrasa
        self.create_subscription(PointCloud2, "/registered_scan", self.cb_lidar, qos_profile_sensor_data,
                                 callback_group=grupo, raw=True)
        # la pose de Point-LIO (miles por segundo): se guarda cruda y se lee a 10 Hz para detectar saltos
        self.odom_raw = None
        self.create_subscription(Odometry, "/state_estimation", self.cb_odom, qos_profile_sensor_data,
                                 callback_group=grupo, raw=True)
        self.create_timer(0.1, self.revisar_pose, callback_group=MutuallyExclusiveCallbackGroup())
        for nombre, fn in (("acostar", self.g.acostar), ("levantar", self.g.levantar),
                           ("bloquear", self.g.bloquear), ("desbloquear", self.g.desbloquear),
                           ("pausa", self.g.alternar_pausa), ("recuperar", self.g.recuperar),
                           ("slam_reiniciado", self.g.slam_reiniciado)):
            self.create_service(Trigger, "/go2/" + nombre, self._srv(nombre, fn), callback_group=grupo)
        self.create_timer(self.g.periodo, self.lazo, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self.estado, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(3.0, self.revisar_evitacion, callback_group=MutuallyExclusiveCallbackGroup())
        self.evita = None
        self.n_cmd = 0
        self.pausa_prev = False

        # Watchdog temporal de Point-LIO:
        # no compara el reloj del sensor contra el reloj ROS del sistema.
        self.ultimo_stamp_odom = None
        self.t_ultimo_avance_odom = None
        self.mandar([(API_STOPMOVE, None)])
        self.get_logger().info("go2_guardian listo: /cmd_vel -> perro (Move a %.0f Hz, StopMove si la orden "
                               "se vence %.2f s). PS4: SHARE+CRUZ acostar, SHARE+TRIANGULO levantar, "
                               "SHARE+CUADRADO bloquear, SHARE+CIRCULO recuperar, CIRCULO pausa"
                               % (1.0 / self.g.periodo, self.g.vigencia))

    def ahora(self):
        return time.monotonic()

    # ------------------------------------------------------------------ entradas
    def cb_cmd(self, m):
        # pathFollower sella /cmd_vel con el timestamp de la odometria usada.
        # Ese reloj puede tener un offset fijo respecto al reloj del sistema.
        # Por eso comprobamos si el timestamp sigue avanzando usando reloj monotónico.
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t_mon = self.ahora()

        with self.lock:
            if st > 0:
                if self.ultimo_stamp_odom is None:
                    # Primer timestamp valido: iniciar el watchdog.
                    self.ultimo_stamp_odom = st
                    self.t_ultimo_avance_odom = t_mon

                elif st > self.ultimo_stamp_odom + 1e-6:
                    # Point-LIO avanzó a una medición más reciente.
                    self.ultimo_stamp_odom = st
                    self.t_ultimo_avance_odom = t_mon

                elif st < self.ultimo_stamp_odom - 1e-3:
                    # Reinicio o salto hacia atrás del reloj del sensor.
                    # Reiniciar la referencia temporal y dar un nuevo periodo de validación.
                    self.ultimo_stamp_odom = st
                    self.t_ultimo_avance_odom = t_mon

            if self.t_ultimo_avance_odom is None:
                edad = None
            else:
                edad = max(0.0, t_mon - self.t_ultimo_avance_odom)

            self.g.orden(
                t_mon,
                m.twist.linear.x,
                m.twist.linear.y,
                m.twist.angular.z,
                edad
            )
            self.n_cmd += 1

    def cb_terreno(self, _m):
        with self.lock:
            self.g.terreno(self.ahora())

    def cb_odom(self, m):
        self.odom_raw = m

    def revisar_pose(self):
        m, self.odom_raw = self.odom_raw, None
        if m is None:
            return
        if isinstance(m, (bytes, bytearray)):
            from rclpy.serialization import deserialize_message
            m = deserialize_message(m, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self.lock:
            self.g.pose(self.ahora(), p.x, p.y, yaw)

    def cb_lidar(self, _m):
        with self.lock:
            self.g.lidar(self.ahora())

    def cb_joy(self, m):
        if m.header.frame_id == "teclado":
            with self.lock:
                self.g.modo_manual(list(m.axes))     # A/D/S/Z/C van en manual (R2): sin anti-giro
                if any(abs(a) > 0.05 for i, a in enumerate(m.axes) if i in (0, 3, 4)):
                    self.g.orden_usuario()
            return
        if m.header.frame_id in JOY_AJENOS:
            with self.lock:
                self.g.orden_usuario()               # meta nueva (RViz, go2v2 ir/parar)
            return
        with self.lock:
            self.g.orden_usuario()                   # el PS4
        with self.lock:
            out = self.g.joystick(self.ahora(), list(m.buttons), list(m.axes))
        self.mandar(out)

    def _srv(self, nombre, fn):
        def cb(_req, res):
            with self.lock:
                out = fn(self.ahora())
            self.mandar(out)
            res.success = True
            res.message = nombre
            return res
        return cb

    def cb_evita(self, m):
        if m.header.identity.api_id != EVITA_GET or m.header.status.code != 0:
            return
        try:
            self.evita = bool(json.loads(m.data).get("enable"))
        except (ValueError, AttributeError):
            pass

    # ------------------------------------------------------------------ salida
    def pedido(self, api, parametros=None, pub=None, sin_respuesta=True):
        r = Request()
        r.header.identity.id = next(self.ids)
        r.header.identity.api_id = api
        r.header.policy.noreply = sin_respuesta
        r.parameter = json.dumps(parametros) if parametros is not None else ""
        (pub or self.pub_req).publish(r)

    def mandar(self, lista):
        for api, par in lista:
            self.pedido(api, par)

    def lazo(self):
        with self.lock:
            out = self.g.paso(self.ahora())
            avisos, self.g.avisos = self.g.avisos, []
            g = self.g
            pausa = g.pausa or g.bloqueado or g.acostado or self.ahora() < g.quieto_hasta
        self.mandar(out)
        for a in avisos:
            self.get_logger().warn(a)
        if pausa != self.pausa_prev:
            # que el seguidor de ruta de CMU tambien frene (y no arranque de golpe al soltar)
            self.pub_stop.publish(Int8(data=15 if pausa else 0))
            self.pausa_prev = pausa

    def revisar_evitacion(self):
        if not self.apagar_evita:
            return
        if self.evita:
            self.get_logger().warn("evasion de obstaculos de Unitree PRENDIDA: la apago (la hace CMU)")
            self.pedido(EVITA_SET, {"enable": False}, pub=self.pub_evita)
        self.pedido(EVITA_GET, None, pub=self.pub_evita, sin_respuesta=False)

    def estado(self):
        with self.lock:
            g = self.g
            d = {"acostado": g.acostado, "bloqueado": g.bloqueado, "pausa": g.pausa,
                 "moviendo": g.moviendo, "orden": [round(v, 3) for v in g.cmd],
                 "orden_hace_s": round(self.ahora() - g.t_cmd, 2) if g.t_cmd > 0 else None,
                 "cmd_hz": self.n_cmd * 2, "move_hz": g.n_move * 2, "stops": g.n_stop,
                 "vencidas": g.n_vencidas, "evasion_unitree": self.evita, "manual": g.manual,
                 "parado_por": g.motivo_parada,
                 "edad_odom_s": None if g.edad_odom is None else round(g.edad_odom, 3),
                 "slam_dudoso": g.slam_dudoso, "salto": g.salto, "vueltas": g.vueltas, "incoherente": g.incoherente,
                 "terreno_hace_s": None if g.t_terreno is None else round(self.ahora() - g.t_terreno, 2),
                 "lidar_hace_s": None if g.t_lidar is None else round(self.ahora() - g.t_lidar, 2)}
            self.n_cmd = 0
            g.n_move = 0
        self.pub_est.publish(String(data=json.dumps(d)))

    def cerrar(self):
        for _ in range(3):
            self.pedido(API_STOPMOVE)
            time.sleep(0.05)


def main():
    # sin el manejador de Ctrl+C de rclpy: el de rclpy apaga ROS ANTES de que se pueda mandar el
    # ultimo StopMove (y el perro podria seguir con la ultima orden). Aca: Ctrl+C -> StopMove -> fin
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init()
    n = NodoGuardian()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            n.get_logger().error("guardian: %r" % (e,))
    finally:
        try:
            n.cerrar()
        except Exception:
            pass
        try:
            ex.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

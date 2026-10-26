"""Corre los NODOS de go2_v2 (sensores, guardian, herramientas) con un ROS de mentira: atrapa errores
de tipeo, atributos mal escritos y flujos rotos sin tener ROS instalado.
python3 v2/pruebas/prueba_nodos_sin_ros.py"""
import json
import os
import sys
import types

import numpy as np

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(AQUI, "..", "ros_ws", "src", "go2_v2"))


# ---------------------------------------------------------------- ROS de mentira
class Msg(object):
    """Mensaje generico: cualquier atributo anidado existe (header.stamp.sec = ...)."""

    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        v = Msg()
        self.__dict__[k] = v
        return v


def tipo(nombre):
    return type(nombre, (Msg,), {})


class Reloj(object):
    t = 1000.0                    # hora de la "laptop" (s)

    @classmethod
    def now(cls):
        return types.SimpleNamespace(nanoseconds=int(cls.t * 1e9), to_msg=lambda: Msg(sec=int(cls.t), nanosec=0))


class Pub(object):
    def __init__(self, topico):
        self.topico = topico
        self.salida = []

    def publish(self, m):
        self.salida.append(m)

    def get_subscription_count(self):
        return 1


class Logger(object):
    def __init__(self):
        self.lineas = []

    def _log(self, nivel):
        def f(txt, **_kw):
            self.lineas.append((nivel, txt))
        return f

    def __getattr__(self, k):
        if k in ("info", "warn", "warning", "error", "debug"):
            return self._log(k)
        raise AttributeError(k)


class Node(object):
    def __init__(self, nombre):
        self.nombre = nombre
        self.pubs = {}
        self.subs = {}
        self.timers = []
        self.servicios = {}
        self.params = {}
        self._logger = Logger()

    def declare_parameter(self, k, v):
        return types.SimpleNamespace(value=self.params.get(k, v))

    def create_publisher(self, _t, topico, _qos):
        p = Pub(topico)
        self.pubs[topico] = p
        return p

    def create_subscription(self, _t, topico, cb, _qos, **kw):
        self.subs[topico] = cb
        return None

    def create_timer(self, periodo, cb, **_kw):
        self.timers.append((periodo, cb))

    def create_service(self, _t, nombre, cb, **_kw):
        self.servicios[nombre] = cb

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return Reloj

    def destroy_node(self):
        pass


def instalar_ros_falso():
    m = {}
    for nombre in ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.callback_groups", "rclpy.executors",
                   "sensor_msgs", "sensor_msgs.msg", "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv",
                   "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "unitree_api",
                   "unitree_api.msg"):
        m[nombre] = types.ModuleType(nombre)
        sys.modules[nombre] = m[nombre]
    m["rclpy.node"].Node = Node
    q = m["rclpy.qos"]
    q.QoSProfile = lambda **kw: kw
    q.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    q.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=2, RELIABLE=1)
    q.qos_profile_sensor_data = {}
    m["rclpy.callback_groups"].MutuallyExclusiveCallbackGroup = object
    m["rclpy.executors"].MultiThreadedExecutor = object
    for mod, tipos in (("sensor_msgs.msg", ("Imu", "PointCloud2", "PointField", "Joy", "JointState")),
                       ("std_msgs.msg", ("String", "Int8")), ("std_srvs.srv", ("Trigger",)),
                       ("geometry_msgs.msg", ("TwistStamped", "PointStamped")),
                       ("nav_msgs.msg", ("Odometry",)), ("unitree_api.msg", ("Request", "Response"))):
        for t in tipos:
            setattr(m[mod], t, tipo(t))
    m["rclpy"].init = lambda *a, **k: None
    m["rclpy"].ok = lambda: True
    m["rclpy"].try_shutdown = lambda: None
    m["rclpy"].spin_once = lambda *a, **k: None


instalar_ros_falso()
from go2_v2 import guardian as G  # noqa: E402
from go2_v2 import sensores as S  # noqa: E402
from go2_v2.guardian_logica import API_MOVE, API_STANDDOWN, API_STOPMOVE, BOTON  # noqa: E402


class Mono(object):
    t = 0.0


def mono():
    return Mono.t


def nube_l1(st_perro, n=3000, seed=0):
    rng = np.random.default_rng(seed)
    dt = np.dtype({"names": ["x", "y", "z", "intensity", "ring", "time"],
                   "formats": [np.float32] * 4 + [np.uint16, np.float32],
                   "offsets": [0, 4, 8, 12, 16, 20], "itemsize": 32})
    a = np.zeros(n, dt)
    a["x"], a["y"], a["z"] = (rng.uniform(-3, 3, n) for _ in range(3))
    a["time"] = np.linspace(0, 0.09, n)
    m = Msg()
    m.header.stamp.sec, m.header.stamp.nanosec = int(st_perro), int(round((st_perro % 1) * 1e9))
    m.height, m.width, m.point_step, m.row_step = 1, n, 32, 32 * n
    m.is_bigendian, m.is_dense = False, True
    m.fields = [Msg(name=k, offset=o, datatype=7 if k != "ring" else 4, count=1)
                for k, o in zip(dt.names, [0, 4, 8, 12, 16, 20])]
    import array
    b = array.array("B")
    b.frombytes(a.tobytes())
    m.data = b
    return m


def imu(st_perro):
    m = Msg()
    m.header.stamp.sec, m.header.stamp.nanosec = int(st_perro), int(round((st_perro % 1) * 1e9))
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = 0.0, 0.0, 0.0, 1.0
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = 0.01, 0.02, 0.5
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = 0.1, 0.0, -9.8
    return m


def probar_sensores():
    S.time.monotonic = mono
    n = S.Sensores()
    nube, cb_imu = n.subs["/utlidar/cloud"], n.subs["/utlidar/imu"]
    desfase_real = 3.5                     # el perro va 3.5 s atrasado
    perro = 500.0
    publicadas = []
    for k in range(300):                   # 1.2 s: IMU a 250 Hz, nube a 10 Hz
        perro += 0.004
        Reloj.t = perro + desfase_real + 0.002 + (0.05 if k % 7 == 0 else 0.0)   # latencias variables
        Mono.t = k * 0.004
        cb_imu(imu(perro))
        if k % 25 == 0:
            nube(nube_l1(perro - 0.1, seed=k))
    pub_nube = n.pubs["/utlidar/transformed_cloud"].salida
    pub_imu = n.pubs["/utlidar/transformed_imu"].salida
    assert n.desfase_ns is not None
    assert abs(n.desfase_ns * 1e-9 - (desfase_real + 0.002)) < 1e-6, n.desfase_ns   # el minimo, no la 1a
    assert pub_imu and pub_nube, (len(pub_imu), len(pub_nube))
    # sellos crecientes y en hora de la laptop
    sts = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in pub_imu]
    assert all(b > a for a, b in zip(sts, sts[1:]))
    assert abs(sts[-1] - (perro + desfase_real + 0.002)) < 1e-3
    c = pub_nube[-1]
    assert c.header.frame_id == "body" and len(c.data) == c.width * c.point_step and c.width > 0
    # IMU repetida o atrasada: se descarta
    antes = len(pub_imu)
    cb_imu(imu(perro))
    cb_imu(imu(perro - 0.5))
    assert len(pub_imu) == antes
    # el reloj del perro salta (reiniciado): error en el log
    Reloj.t += 0.004
    cb_imu(imu(10.0))
    assert any(niv == "error" and "SALTO" in t for niv, t in n.get_logger().lineas)
    for _, cb in n.timers:
        cb()
    est = json.loads(n.pubs["/go2/sensores_estado"].salida[-1].data)
    assert est["desfase_s"] is not None
    publicadas.append(len(pub_nube))
    print("sensores: desfase %.3f s (minimo), %d nubes, %d imu, estado %s" % (n.desfase_ns * 1e-9, len(pub_nube),
                                                                              len(pub_imu), sorted(est)))


def probar_guardian():
    G.time.monotonic = mono
    Mono.t = 100.0
    Reloj.t = 2000.0
    n = G.NodoGuardian()
    req = n.pubs["/api/sport/request"].salida
    assert req and req[0].header.identity.api_id == API_STOPMOVE           # StopMove al arrancar
    lazo = [cb for p, cb in n.timers if abs(p - 0.02) < 1e-9][0]

    def cmd(vx, edad=0.1):
        m = Msg()
        st = Reloj.t - edad
        m.header.stamp.sec, m.header.stamp.nanosec = int(st), int(round((st % 1) * 1e9))
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, 0.0, 0.0
        n.subs["/cmd_vel"](m)

    def correr(seg, vx=None, terreno=True):
        for _ in range(int(round(seg / 0.02))):
            Mono.t += 0.02
            Reloj.t += 0.02
            if terreno:
                n.subs["/terrain_map"](b"\x00")
                n.subs["/registered_scan"](b"\x00")
            if vx is not None:
                cmd(vx)
            lazo()

    del req[:]
    correr(1.0, vx=0.5)                       # con suavizado llega a 0.5 m/s en ~0.6 s (rampa)
    apis = [r.header.identity.api_id for r in req]
    assert apis.count(API_MOVE) >= 40, apis
    assert json.loads(req[0].parameter)["x"] < 0.05                     # arranca suave
    assert json.loads(req[-1].parameter) == {"x": 0.5, "y": 0.0, "z": 0.0}
    # el planificador se calla -> StopMove
    del req[:]
    correr(0.6)
    assert API_STOPMOVE in [r.header.identity.api_id for r in req]
    # joy de RViz / go2v2: ignorado; joy del control: SHARE + CRUZ = acostar
    j = Msg()
    j.header.frame_id = "goalpoint_tool"
    j.buttons = [0] * 13
    j.buttons[BOTON["share"]] = 1
    j.buttons[BOTON["cruz"]] = 1
    j.axes = [0.0] * 8
    n.subs["/joy"](j)
    assert not n.g.acostado
    j.header.frame_id = "joy"
    j.buttons = [0] * 13
    j.buttons[BOTON["share"]] = 1
    n.subs["/joy"](j)
    j2 = Msg()
    j2.header.frame_id = "joy"
    j2.axes = [0.0] * 8
    j2.buttons = list(j.buttons)
    j2.buttons[BOTON["cruz"]] = 1
    n.subs["/joy"](j2)
    assert n.g.acostado
    del req[:]
    correr(1.0, vx=0.5)
    apis = [r.header.identity.api_id for r in req]
    assert API_STANDDOWN in apis and API_MOVE not in apis, apis
    # /stop a CMU mientras esta acostado
    assert n.pubs["/stop"].salida and n.pubs["/stop"].salida[-1].data == 15
    # servicio levantar
    res = n.servicios["/go2/levantar"](None, Msg())
    assert res.success and not n.g.acostado
    assert n.servicios["/go2/slam_reiniciado"](None, Msg()).success
    correr(3.5, vx=0.5)
    assert n.pubs["/stop"].salida[-1].data == 0
    for p, cb in n.timers:
        if p != 0.02:
            cb()
    est = json.loads(n.pubs["/go2/guardian_estado"].salida[-1].data)
    assert "parado_por" in est and "edad_odom_s" in est and "lidar_hace_s" in est
    # sin nube del lidar (Point-LIO solo con IMU): no camina aunque el terreno siga llegando
    for _ in range(40):
        Mono.t += 0.02
        Reloj.t += 0.02
        n.subs["/terrain_map"](b"\x00")
        cmd(0.5)
        lazo()
    del req[:]
    for _ in range(10):
        Mono.t += 0.02
        Reloj.t += 0.02
        n.subs["/terrain_map"](b"\x00")
        cmd(0.5)
        lazo()
    assert API_MOVE not in [r.header.identity.api_id for r in req], "camina sin lidar"
    assert "lidar" in (n.g.motivo_parada or "")
    print("guardian: Move/StopMove, joy ajenos ignorados, acostar/levantar, /stop a CMU, estado %s" % sorted(est))


def probar_herramientas(tmp):
    from go2_v2 import herramientas as H
    H.ARCHIVO = os.path.join(tmp, "puntos.yaml")
    H.time.monotonic = mono
    creados = []
    orig = H.Herramientas.__init__

    def init(self):
        orig(self)
        creados.append(self)
        o = Msg()
        o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = 1.5, -2.0, 0.3
        o.pose.pose.orientation.x = o.pose.pose.orientation.y = o.pose.pose.orientation.z = 0.0
        o.pose.pose.orientation.w = 1.0
        o.header.stamp.sec, o.header.stamp.nanosec = int(Reloj.t), 0
        self.subs["/state_estimation"](o)
    H.Herramientas.__init__ = init

    def girar(self, seg, hasta=None):
        Mono.t += seg
        return hasta() if hasta else False
    H.Herramientas.girar = girar
    sys.argv = ["x", "guardar", "cocina"]
    assert H.main() == 0
    assert H.leer_puntos() == {"cocina": (1.5, -2.0)}
    sys.argv = ["x", "ir", "cocina"]
    assert H.main() == 0
    n = creados[-1]
    g = n.pubs["/goal_point"].salida
    assert g and g[-1].point.x == 1.5 and g[-1].header.frame_id == "map"
    assert n.pubs["/joy"].salida[-1].axes[2] == -1.0 and n.pubs["/joy"].salida[-1].header.frame_id == "go2v2"
    sys.argv = ["x", "ir", "3", "4"]
    assert H.main() == 0
    sys.argv = ["x", "ir", "nada"]
    assert H.main() == 1
    sys.argv = ["x", "parar"]
    assert H.main() == 0
    assert creados[-1].pubs["/joy"].salida[-1].axes[2] == 0.0
    sys.argv = ["x", "borrar", "cocina"]
    assert H.main() == 0 and H.leer_puntos() == {}
    print("herramientas: guardar / ir NOMBRE / ir X Y / parar / borrar")


def probar_fantasmas_y_mapa(tmp):
    from go2_v2 import fantasmas as F
    from go2_v2 import mapa as MP
    n = F.Fantasmas()
    sal = n.pubs["/registered_scan_limpia"].salida
    # sin odometria todavia: pasa todo tal cual
    nube = nube_l1(10.0, n=500, seed=1)
    n.subs["/registered_scan"](nube)
    assert sal[-1] is nube
    odo = Msg()
    odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z = 0.0, 0.0, 0.4
    n.subs["/state_estimation"](odo)
    rng = np.random.default_rng(3)
    for k in range(6):
        m = nube_l1(10.0 + 0.1 * k, n=500, seed=1)              # lo mismo cada vez: es "real"
        a = np.frombuffer(bytes(m.data), np.uint8).reshape(500, 32).copy()
        fant = rng.uniform(0.5, 1.5, (1, 3)).astype(np.float32)
        a[-1, 0:12] = fant.view(np.uint8)                       # el ultimo punto: un fantasma suelto
        import array
        b = array.array("B")
        b.frombytes(a.tobytes())
        m.data = b
        n.subs["/registered_scan"](m)
    out = sal[-1]
    assert out.width < 500 and len(out.data) == out.width * out.point_step, out.width
    assert out.header is m.header and out.fields is m.fields
    for _, cb in n.timers:
        cb()
    est = json.loads(n.pubs["/go2/fantasmas_estado"].salida[-1].data)
    assert est["fantasmas"] >= 1 and est["activo"]

    mp = MP.Mapa()
    mp.carpeta = tmp
    for k in range(3):
        mp.subs["/registered_scan"](nube_l1(5.0 + k, n=300, seed=k))
    for _, cb in mp.timers:
        cb()
    pm = mp.pubs["/go2/mapa"].salida[-1]
    assert pm.width > 0 and len(pm.data) == 12 * pm.width
    res = mp.servicios["/go2/guardar_mapa"](None, Msg())
    assert res.success and os.path.exists(res.message.split("-> ")[1]), res.message
    res = mp.servicios["/go2/borrar_mapa"](None, Msg())
    res = mp.servicios["/go2/guardar_mapa"](None, Msg())
    assert not res.success
    print("fantasmas: %d fantasmas sacados, nube intacta; mapa: publica, guarda .pcd, borra" % est["fantasmas"])


def probar_modelo():
    from go2_v2 import modelo as M
    M.time.monotonic = mono
    Mono.t = 50.0
    n = M.Modelo()
    pub = n.pubs["/joint_states"].salida
    for _, cb in n.timers:
        cb()
    assert pub and pub[-1].position == M.DE_PIE and len(pub[-1].name) == 12   # sin perro: de pie
    # con el perro (si unitree_go existe se suscribe; aca se simula el callback)
    low = Msg()
    low.motor_state = [Msg(q=0.1 * (i + 1)) for i in range(20)]
    Mono.t += 0.1
    n.cb_low(low)
    assert abs(pub[-1].position[11] - 1.2) < 1e-9
    print("modelo: de pie sin datos, angulos reales con /lf/lowstate")


if __name__ == "__main__":
    import tempfile
    probar_modelo()
    probar_sensores()
    probar_guardian()
    with tempfile.TemporaryDirectory() as d:
        probar_herramientas(d)
    with tempfile.TemporaryDirectory() as d:
        probar_fantasmas_y_mapa(d)
    print("OK nodos")

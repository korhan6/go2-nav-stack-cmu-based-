"""El nodo go2_metas entero con un ROS simulado: python3 pruebas/prueba_nodo_metas.py"""
import sys, os, types, json, tempfile, array
W = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, W+"/pruebas"); sys.argv=["x"]
import prueba_nodos_sin_ros as F          # instala el ROS falso
import numpy as np
class _Path(F.Msg):
    def __init__(self, **kw):
        super().__init__(**kw); self.poses = []
sys.modules["nav_msgs.msg"].Path = _Path
for mod, tipos in (("nav_msgs.msg", ("OccupancyGrid",)), ("geometry_msgs.msg", ("PoseStamped",)), ("std_msgs.msg", ("Empty",))):
    for t in tipos: setattr(sys.modules[mod], t, F.tipo(t))
ser = types.ModuleType("rclpy.serialization"); ser.deserialize_message = lambda m, t: m; sys.modules["rclpy.serialization"] = ser
cap = {}
sys.modules["rclpy"].spin = lambda n: (cap.__setitem__("n", n), (_ for _ in ()).throw(KeyboardInterrupt()))
from go2_v2 import metas as M
d = tempfile.mkdtemp()
# guardar primero un mapa para probar la carga
from go2_v2.ruta import Grilla
g = Grilla(res=0.1, lado=60.0); g.c[g.celda(5, 5)] = 1; g.guardar(os.path.join(d, "grilla_ultima.npz"))
orig_init = F.Node.__init__
def init(self, nombre):
    orig_init(self, nombre); self.params = {"carpeta": d, "cargar_mapa": "ultimo"}
F.Node.__init__ = init
M.main()
n = cap["n"]
assert n.grilla.c[n.grilla.celda(5, 5)] == 1, "no cargo el mapa"
def nube(pts):
    a = np.zeros(len(pts), dtype=[("x","f4"),("y","f4"),("z","f4"),("intensity","f4")]); a["x"],a["y"],a["z"],a["intensity"]=pts.T
    m = F.Msg(); m.width, m.height, m.point_step = len(pts), 1, 16
    m.fields = [F.Msg(name=k, offset=o, datatype=7, count=1) for k, o in zip(("x","y","z","intensity"), (0,4,8,12))]
    b = array.array("B"); b.frombytes(a.tobytes()); m.data = b; m.is_bigendian=False; m.is_dense=True; return m
def odom(x, y, yaw):
    o = F.Msg(); o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = x, y, 0.0
    o.pose.pose.orientation.x = o.pose.pose.orientation.y = 0.0
    o.pose.pose.orientation.z, o.pose.pose.orientation.w = np.sin(yaw/2), np.cos(yaw/2); return o
paso = [cb for p, cb in n.timers if abs(p-0.2)<1e-9][0]
xs, ys = np.meshgrid(np.arange(-1, 3, 0.05), np.arange(-1, 1, 0.05))
piso = np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, -0.35), np.zeros(xs.size)]
fant = np.c_[np.full(10, 1.0), np.linspace(-0.05, 0.05, 10), np.full(10, 0.0), np.full(10, 0.35)]
pts = np.vstack([piso, fant])
n.subs["/terrain_map"](nube(pts))
assert n.pubs["/terrain_map_go2"].salida[-1].width == len(pts)            # todavia no piso nada
# el perro camina por y=0 de x=0.3 a x=2.3 (lidar 0.3 adelante del centro)
for k in range(30):
    n.subs["/state_estimation"](odom(0.3 + k*0.07, 0.0, 0.0)); paso()
n.subs["/terrain_map"](nube(pts))
out = n.pubs["/terrain_map_go2"].salida[-1]
assert out.width == len(piso), out.width                                    # el fantasma se borro
# meta con el planificador propio -> way_point + ruta
gp = F.Msg(); gp.point.x, gp.point.y, gp.point.z = 2.5, 0.0, 0.0
n.subs["/goal_point"](gp)
n.subs["/terrain_map"](nube(pts)); n.subs["/state_estimation"](odom(0.3, 0.0, 0.0)); paso()
assert n.pubs["/way_point"].salida, "sin way_point"
assert n.pubs["/go2/ruta"].salida[-1].poses
for p, cb in n.timers:
    if p in (0.5, 1.0): cb()
est = json.loads(n.pubs["/go2/metas_estado"].salida[-1].data)
gr = n.pubs["/go2/grilla"].salida[-1]
assert len(gr.data) == gr.info.width * gr.info.height
res = n.servicios["/go2/guardar_grilla"](None, F.Msg())
assert res.success and os.path.exists(os.path.join(d, "grilla_ultima.npz"))
assert n.servicios["/go2/olvidar_pisadas"](None, F.Msg()).success and not n.grilla.pisadas.any()
print("nodo metas: carga mapa, borra el fantasma pisado, ruta + way_point, grilla RViz, guarda. estado:", est)

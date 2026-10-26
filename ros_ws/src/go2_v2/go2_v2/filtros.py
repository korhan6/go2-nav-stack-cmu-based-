"""Matematica de nubes sin ROS (se prueba sola: pruebas/prueba_filtros.py).

  * Filtro de fantasmas: un punto del lidar se queda solo si en su vecindario (cubo de 3x3x3 voxeles)
    hay suficientes puntos en las ultimas nubes. Lo real (paredes, muebles, gente, el piso) se repite
    en nube tras nube en el mismo lugar del mapa; el ruido del L1 (puntos sueltos, reflejos, polvo)
    aparece una vez y no se repite. Solo se filtra cerca del perro (radio): lejos el L1 es ralo de
    por si y el planificador local no mira tan lejos.
  * Mapa: voxeles unicos (para juntar lo que ve el lidar mientras camina) y archivo PCD.
"""
import numpy as np

_BITS = 21
_OFF = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1
_VECINOS = np.array([(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)],
                    dtype=np.int64)


def indices(xyz, tam):
    """(n, 3) metros -> (n, 3) indices enteros de voxel de lado 'tam'."""
    return np.floor(np.asarray(xyz, np.float64) / tam).astype(np.int64)


def codificar(ijk):
    """(n, 3) indices -> (n,) claves int64 (cada eje +-1 millon de voxeles: +-50 km a 5 cm)."""
    ijk = np.asarray(ijk, np.int64) + _OFF
    return (ijk[:, 0] << (2 * _BITS)) | (ijk[:, 1] << _BITS) | ijk[:, 2]


def decodificar(claves):
    c = np.asarray(claves, np.int64)
    return np.stack([(c >> (2 * _BITS)) & _MASK, (c >> _BITS) & _MASK, c & _MASK], axis=1) - _OFF


def contar_vecindario(ijk, claves_ventana, cuentas_ventana):
    """Para cada voxel de 'ijk', cuantos puntos hay en su cubo 3x3x3 segun la ventana
    (claves_ventana ordenadas y unicas, con su cuenta de puntos)."""
    total = np.zeros(len(ijk), np.int64)
    if len(claves_ventana) == 0:
        return total
    for d in _VECINOS:
        k = codificar(ijk + d)
        i = np.searchsorted(claves_ventana, k)
        i[i >= len(claves_ventana)] = 0
        hay = claves_ventana[i] == k
        total[hay] += cuentas_ventana[i[hay]]
    return total


class FiltroFantasmas(object):
    """Guarda las ultimas nubes (ya en el marco del mapa) y decide que puntos de la nube nueva son
    reales. tam: lado del voxel (m). min_puntos: puntos minimos en el cubo 3x3x3 contando la ventana
    (incluida la nube nueva). ventana: s de nubes que se recuerdan. radio: solo se filtra a menos de
    'radio' m del perro (en el plano). z_min_rel: solo se filtra lo que esta mas arriba que
    (altura del lidar + z_min_rel). EL PISO NUNCA SE FILTRA: el L1 lo ve ralo a 1-2 m y si se le
    quitaran puntos, el analisis de terreno de CMU lo tomaria como 'sin datos' = pared falsa."""

    def __init__(self, tam=0.1, min_puntos=5, ventana=0.5, radio=4.0, z_min_rel=-0.2):
        self.z_min_rel = float(z_min_rel)
        self.tam = float(tam)
        self.min_puntos = int(min_puntos)
        self.ventana = float(ventana)
        self.radio = float(radio)
        self.hist = []                  # [(t, claves de voxel de cada punto)]

    def filtrar(self, t, xyz, centro):
        """xyz (n, 3) en el mapa, centro (x, y, z) del lidar (/state_estimation). Devuelve la mascara de puntos a conservar."""
        xyz = np.asarray(xyz, np.float64)
        n = len(xyz)
        if n == 0:
            return np.zeros(0, bool)
        ijk = indices(xyz, self.tam)
        claves = codificar(ijk)
        self.hist = [(tt, c) for tt, c in self.hist if t - tt <= self.ventana and tt <= t]
        self.hist.append((t, claves))
        todas = np.concatenate([c for _, c in self.hist])
        uk, cuentas = np.unique(todas, return_counts=True)
        cerca = ((np.hypot(xyz[:, 0] - centro[0], xyz[:, 1] - centro[1]) < self.radio)
                 & (xyz[:, 2] - centro[2] > self.z_min_rel))
        ok = np.ones(n, bool)
        if cerca.any():
            ok[cerca] = contar_vecindario(ijk[cerca], uk, cuentas) >= self.min_puntos
        return ok


class AcumuladorMapa(object):
    """Junta nubes en voxeles unicos de lado 'tam' (el mapa que se arma mientras camina)."""

    def __init__(self, tam=0.05, max_voxeles=5_000_000):
        self.tam = float(tam)
        self.max_voxeles = int(max_voxeles)
        self.claves = np.zeros(0, np.int64)
        self.pendientes = []
        self.lleno = False

    def agregar(self, xyz):
        if len(xyz):
            self.pendientes.append(codificar(indices(xyz, self.tam)))

    def consolidar(self):
        if not self.pendientes:
            return len(self.claves)
        nuevas = np.concatenate(self.pendientes)
        self.pendientes = []
        if self.lleno:
            return len(self.claves)
        self.claves = np.unique(np.concatenate([self.claves, nuevas]))
        if len(self.claves) > self.max_voxeles:
            self.claves = self.claves[:self.max_voxeles]
            self.lleno = True
        return len(self.claves)

    def puntos(self, tam_salida=None):
        """Centros de los voxeles (m, float32). tam_salida > tam: version mas liviana (para RViz)."""
        self.consolidar()
        if tam_salida and tam_salida > self.tam:
            xyz = (decodificar(self.claves) + 0.5) * self.tam
            claves = np.unique(codificar(indices(xyz, tam_salida)))
            return ((decodificar(claves) + 0.5) * tam_salida).astype(np.float32)
        return ((decodificar(self.claves) + 0.5) * self.tam).astype(np.float32)

    def borrar(self):
        self.claves = np.zeros(0, np.int64)
        self.pendientes = []
        self.lleno = False


def escribir_pcd(ruta, xyz):
    """PCD binario (x y z float32): lo abren CloudCompare, PCL, Open3D, RViz (con un nodo)..."""
    xyz = np.ascontiguousarray(np.asarray(xyz, np.float32).reshape(-1, 3))
    n = len(xyz)
    cab = ("# .PCD v0.7 - go2v2\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
           "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (n, n))
    with open(ruta, "wb") as f:
        f.write(cab.encode("ascii"))
        f.write(xyz.tobytes())
    return n


def xyz_de_nube(data, n, paso, offsets):
    """Bytes de un PointCloud2 (n puntos de 'paso' bytes, x y z float32 en 'offsets') -> (n, 3)."""
    crudo = np.frombuffer(data, dtype=np.uint8, count=n * paso).reshape(n, paso)
    xyz = np.empty((n, 3), np.float32)
    for i, o in enumerate(offsets):
        xyz[:, i] = np.ascontiguousarray(crudo[:, o:o + 4]).view(np.float32)[:, 0]
    return xyz, crudo

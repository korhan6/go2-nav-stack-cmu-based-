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


def calibrar_fantasmas(nubes, centros, tam=0.1, radio=4.0, z_min_rel=-0.2, ventanas=(0.5, 0.75, 1.0),
                       minimos=range(2, 9), guardar_real=0.97, persistencia=0.15):
    """Elige min_puntos y ventana del filtro de fantasmas con lo que el lidar vio quieto / girando en el sitio.

    nubes: [(t, xyz (n, 3) en el mapa)], centros: [(x, y, z)] del lidar en cada nube.
      REAL     = punto cuyo voxel aparece en muchas nubes (>= 'persistencia' de ellas, minimo 3): paredes,
                 muebles, patas. Tiene que QUEDAR.
      FANTASMA = punto cuyo voxel aparece en UNA sola nube y sin nada real alrededor: ruido. Tiene que IRSE.
    Para cada ventana se cuenta el vecindario de cada punto como lo hace el filtro, y se prueban todos los
    minimos: gana el que saca mas fantasmas guardando al menos 'guardar_real' de lo real (a igualdad, la
    ventana mas corta: reacciona antes). Tope 8: una persona CAMINANDO deja menos puntos en el mismo lugar que
    algo quieto, y un filtro mas exigente la podria borrar. Devuelve un dict con min_puntos, ventana y como queda."""
    n = len(nubes)
    if n < 10:
        return None
    claves = []
    for (t, xyz), c in zip(nubes, centros):
        xyz = np.asarray(xyz, np.float64)
        dentro = ((np.hypot(xyz[:, 0] - c[0], xyz[:, 1] - c[1]) < radio) & (xyz[:, 2] - c[2] > z_min_rel))
        claves.append((t, codificar(indices(xyz, tam)), dentro, indices(xyz, tam)))
    # en cuantas nubes aparece cada voxel
    por_nube = np.concatenate([np.unique(k) for _, k, _, _ in claves])
    uk, veces = np.unique(por_nube, return_counts=True)
    umbral_real = max(3, int(np.ceil(persistencia * n)))

    def apariciones(k):
        i = np.searchsorted(uk, k)
        i[i >= len(uk)] = 0
        return np.where(uk[i] == k, veces[i], 0)
    real_uk = uk[veces >= umbral_real]
    real_cnt = np.ones(len(real_uk), np.int64)
    etiquetas = []
    for t, k, dentro, ijk in claves:
        ap = apariciones(k)
        real = dentro & (ap >= umbral_real)
        aislado = contar_vecindario(ijk, real_uk, real_cnt) == 0
        fantasma = dentro & (ap == 1) & aislado
        etiquetas.append((real, fantasma))
    tot_real = sum(int(r.sum()) for r, _ in etiquetas)
    tot_fant = sum(int(f.sum()) for _, f in etiquetas)
    if tot_real == 0:
        return None
    resultados = []
    for v in ventanas:
        cr, cf = [], []
        hist = []
        for (t, k, dentro, ijk), (real, fant) in zip(claves, etiquetas):
            hist = [(tt, kk) for tt, kk in hist if t - tt <= v and tt <= t]
            hist.append((t, k))
            uk2, cu2 = np.unique(np.concatenate([kk for _, kk in hist]), return_counts=True)
            cnt = contar_vecindario(ijk, uk2, cu2)
            cr.append(cnt[real])
            cf.append(cnt[fant])
        cr = np.concatenate(cr) if cr else np.zeros(0, np.int64)
        cf = np.concatenate(cf) if cf else np.zeros(0, np.int64)
        for m in minimos:
            guarda = float((cr >= m).mean()) if len(cr) else 1.0
            quita = float((cf < m).mean()) if len(cf) else 0.0
            resultados.append({"ventana": float(v), "min_puntos": int(m), "guarda_real": round(guarda, 4),
                               "quita_fantasmas": round(quita, 4)})
    buenos = [r for r in resultados if r["guarda_real"] >= guardar_real]
    if buenos:
        # los que sacan casi tantos fantasmas como el mejor (0.5 %): de esos, el que mas real guarda
        tope = max(r["quita_fantasmas"] for r in buenos)
        casi = [r for r in buenos if r["quita_fantasmas"] >= tope - 0.005]
        mejor = max(casi, key=lambda r: (r["guarda_real"], -r["ventana"], -r["min_puntos"]))
    else:
        mejor = max(resultados, key=lambda r: (r["guarda_real"], r["quita_fantasmas"]))
        mejor = dict(mejor, aviso="no llegue a guardar %.0f %% de lo real: uso el mas suave" % (100 * guardar_real))
    mejor.update({"nubes": n, "puntos_reales": tot_real, "fantasmas_vistos": tot_fant})
    return mejor

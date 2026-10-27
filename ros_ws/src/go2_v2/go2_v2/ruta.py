"""Planificador de rutas propio (Fase 1 del documento "GO2 Adaptive Terrain-Aware Route Planner"):
A* sobre una grilla 2D que se arma con el mapa de terreno. Sin ROS: se prueba sola
(pruebas/prueba_ruta.py).

Por que (grabacion 26/10 14:56): con la meta del otro lado de la puerta, FAR planner mandaba
way_points a (1.8, 0.9), despues a (-0.6, -1.8), despues para atras... y el perro giraba. A* busca el
camino mas corto que ENTRA por donde cabe el perro, y prefiere ir por el centro de los pasos angostos.

Grilla (celdas de 'res' m, fija en el marco del mapa, centrada en el origen):
  -1 desconocido (se puede pasar, cuesta un poco mas)
   0 libre (se vio piso)
   1 obstaculo (algo mas alto que 'altura' sobre el piso). Los 'huecos' (piso mas bajo) ya no cuentan
Cada nube del terreno reescribe las celdas que toca (lo que se movio se limpia solo).
"""
import heapq
import math

import numpy as np

DESCONOCIDO, LIBRE, OBSTACULO = -1, 0, 1


class Grilla(object):
    def __init__(self, res=0.1, lado=60.0, altura=0.3, hueco=0.2, radio=0.25, margen=0.5):
        self.res = float(res)
        self.n = int(round(lado / res))
        self.c = np.full((self.n, self.n), DESCONOCIDO, np.int8)
        self.altura = altura          # m sobre el piso: obstaculo
        self.hueco = hueco            # m bajo el piso del perro: hueco (escalera abajo)
        self.radio = radio            # m: medio ancho del perro + algo (celdas mas cerca de un obstaculo: prohibidas)
        self.margen = margen          # m: hasta aca cerca de obstaculos cuesta mas (va por el centro)
        # PISADAS: celdas por donde paso el cuerpo del perro. Si paso, ahi no hay nada solido: lo que el
        # lidar crea ver ahi (fantasmas) se borra y no vuelve (ver 'filtrar_pisadas')
        self.pisadas = np.zeros((self.n, self.n), bool)
        # FANTASMAS CONFIRMADOS: celdas donde el cuerpo del perro estuvo parado MIENTRAS el mapa decia que
        # ahi habia un obstaculo: eso era mentira (un fantasma). Solo ESTAS se borran despues. 26/10: antes se
        # borraba todo lo bajo (< 0.7 m) en CUALQUIER celda pisada, y si una persona se paraba donde el perro
        # ya habia caminado, sus piernas desaparecian y el perro se le iba encima
        self.fantasmas = np.zeros((self.n, self.n), bool)

    def celda(self, x, y):
        return (int(math.floor(x / self.res)) + self.n // 2, int(math.floor(y / self.res)) + self.n // 2)

    def centro(self, i, j):
        return ((i - self.n // 2 + 0.5) * self.res, (j - self.n // 2 + 0.5) * self.res)

    def actualizar(self, xyz_h, piso_z):
        """xyz_h: (n, 4) x, y, z, altura sobre el piso local (la 'intensity' del mapa de terreno de CMU).
        piso_z: z del piso donde esta parado el perro (para detectar huecos)."""
        if len(xyz_h) == 0:
            return
        p = np.asarray(xyz_h, np.float64)
        i = np.floor(p[:, 0] / self.res).astype(np.int64) + self.n // 2
        j = np.floor(p[:, 1] / self.res).astype(np.int64) + self.n // 2
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        i, j, p = i[ok], j[ok], p[ok]
        # 26/10: sin 'hueco' (piso mas bajo = obstaculo): marcaba negro medio cuarto y no avanzaba. Solo lo
        # que sobresale mas de 'altura' del piso es obstaculo (OJO: una escalera para abajo ya no frena)
        obst = p[:, 3] > self.altura
        k = i * self.n + j
        tocadas = np.unique(k)
        con_obst = np.unique(k[obst])
        plano = self.c.reshape(-1)
        plano[tocadas] = LIBRE
        plano[con_obst] = OBSTACULO
        self.c[self.fantasmas & (self.c == OBSTACULO)] = LIBRE

    def pisar(self, x, y, yaw, largo=0.6, ancho=0.36, nube=None, umbral=0.1, alto_max=0.7, achicar=0.05):
        """El cuerpo del perro (rectangulo largo x ancho centrado en x, y, girado yaw) ocupa estas celdas.
        nube (n, 4: x y z altura): el mapa de terreno de AHORA. Los 'obstaculos' (umbral < altura < alto_max)
        que caen bajo el cuerpo (achicado 'achicar' m, para no tocar lo que esta pegado al costado) son
        fantasmas confirmados: esas celdas se borran desde ahora (hasta olvidar_pisadas)."""
        r = 0.5 * math.hypot(largo, ancho)
        i0, j0 = self.celda(x - r, y - r)
        i1, j1 = self.celda(x + r, y + r)
        i0, j0 = max(0, i0), max(0, j0)
        i1, j1 = min(self.n - 1, i1), min(self.n - 1, j1)
        if i0 > i1 or j0 > j1:
            return 0
        ii, jj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
        cx = (ii - self.n // 2 + 0.5) * self.res - x
        cy = (jj - self.n // 2 + 0.5) * self.res - y
        fx = cx * math.cos(yaw) + cy * math.sin(yaw)
        fy = -cx * math.sin(yaw) + cy * math.cos(yaw)
        dentro = (np.abs(fx) <= largo / 2) & (np.abs(fy) <= ancho / 2)
        self.pisadas[i0:i1 + 1, j0:j1 + 1] |= dentro
        self.c[i0:i1 + 1, j0:j1 + 1][dentro & (self.c[i0:i1 + 1, j0:j1 + 1] == OBSTACULO)] = LIBRE
        if nube is None or len(nube) == 0:
            return 0
        p = np.asarray(nube)
        p = p[(p[:, 3] > umbral) & (p[:, 3] < alto_max)]
        if len(p) == 0:
            return 0
        # cada punto en el marco del cuerpo: solo los que caen DEBAJO del cuerpo (achicado)
        dx, dy = p[:, 0] - x, p[:, 1] - y
        px = dx * math.cos(yaw) + dy * math.sin(yaw)
        py = -dx * math.sin(yaw) + dy * math.cos(yaw)
        sel = (np.abs(px) <= largo / 2 - achicar) & (np.abs(py) <= ancho / 2 - achicar)
        if not sel.any():
            return 0
        pi = np.floor(p[sel, 0] / self.res).astype(np.int64) + self.n // 2
        pj = np.floor(p[sel, 1] / self.res).astype(np.int64) + self.n // 2
        ok = (pi >= 0) & (pi < self.n) & (pj >= 0) & (pj < self.n)
        pi, pj = pi[ok], pj[ok]
        nuevas = int((~self.fantasmas[pi, pj]).sum())
        self.fantasmas[pi, pj] = True
        return nuevas

    def filtrar_pisadas(self, xyz_h, alto_max=0.7, umbral=0.1):
        """Mascara de los puntos del mapa de terreno que se QUEDAN: se sacan los 'obstaculos' (mas de
        'umbral' sobre el piso, menos de alto_max) que caen en celdas de FANTASMAS CONFIRMADOS (ver pisar).
        Lo que aparece despues en una celda solo pisada (una persona que se paro ahi) se ve normal."""
        p = np.asarray(xyz_h)
        if len(p) == 0:
            return np.zeros(0, bool)
        i = np.floor(p[:, 0] / self.res).astype(np.int64) + self.n // 2
        j = np.floor(p[:, 1] / self.res).astype(np.int64) + self.n // 2
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        pis = np.zeros(len(p), bool)
        pis[ok] = self.fantasmas[i[ok], j[ok]]
        return ~(pis & (p[:, 3] > umbral) & (p[:, 3] < alto_max))

    def olvidar_pisadas(self):
        self.pisadas[:] = False
        self.fantasmas[:] = False

    def guardar(self, ruta):
        np.savez_compressed(ruta, c=self.c, pisadas=self.pisadas, fantasmas=self.fantasmas, res=self.res)

    def cargar(self, ruta):
        d = np.load(ruta)
        if d["c"].shape != self.c.shape or abs(float(d["res"]) - self.res) > 1e-9:
            raise ValueError("el mapa guardado tiene otro tamano/resolucion")
        self.c[:] = d["c"]
        self.pisadas[:] = d["pisadas"]
        # mapas viejos (sin 'fantasmas'): no se borra nada (lo seguro)
        self.fantasmas[:] = d["fantasmas"] if "fantasmas" in d.files else False

    def costos(self, i0, i1, j0, j1):
        """Ventana [i0:i1, j0:j1]: (prohibido bool, costo extra float)."""
        sub = self.c[i0:i1, j0:j1]
        obs = sub == OBSTACULO
        # distancia (en celdas) al obstaculo mas cercano, por barridos (sin scipy)
        INF = 10 ** 6
        d = np.where(obs, 0, INF).astype(np.int64)
        r_max = int(math.ceil(self.margen / self.res)) + 1
        for _ in range(r_max):
            dn = d.copy()
            dn[1:, :] = np.minimum(dn[1:, :], d[:-1, :] + 1)
            dn[:-1, :] = np.minimum(dn[:-1, :], d[1:, :] + 1)
            dn[:, 1:] = np.minimum(dn[:, 1:], d[:, :-1] + 1)
            dn[:, :-1] = np.minimum(dn[:, :-1], d[:, 1:] + 1)
            if np.array_equal(dn, d):
                break
            d = dn
        dist = d * self.res
        prohibido = dist < self.radio
        extra = np.where(dist < self.margen, (self.margen - dist) / self.margen * 3.0, 0.0)
        extra = extra + np.where(sub == DESCONOCIDO, 0.5, 0.0)
        return prohibido, extra

    def planificar(self, inicio, meta, borde=3.0, lo_mas_cerca=False):
        """A* de inicio a meta (x, y en el mapa). Devuelve la lista de puntos (x, y) o None si no hay
        camino. El perro puede estar pegado a algo (se libera alrededor del inicio); si la meta cae en
        un lugar prohibido, se usa la celda libre mas cercana (hasta 0.6 m).
        lo_mas_cerca: si no hay camino a la meta (esta en la escalera, contra una pared, del otro lado de
        algo cerrado), devuelve el camino al punto ALCANZABLE mas cerca de la meta."""
        si, sj = self.celda(*inicio)
        gi, gj = self.celda(*meta)
        b = int(round(borde / self.res))
        i0, i1 = max(0, min(si, gi) - b), min(self.n, max(si, gi) + b + 1)
        j0, j1 = max(0, min(sj, gj) - b), min(self.n, max(sj, gj) + b + 1)
        if not (i0 <= si < i1 and j0 <= sj < j1 and i0 <= gi < i1 and j0 <= gj < j1):
            return None
        prohibido, extra = self.costos(i0, i1, j0, j1)
        H, W = prohibido.shape
        s = (si - i0, sj - j0)
        g = (gi - i0, gj - j0)
        rl = int(math.ceil(self.radio / self.res))       # el perro puede arrancar pegado a algo
        prohibido[max(0, s[0] - rl):s[0] + rl + 1, max(0, s[1] - rl):s[1] + rl + 1] &= \
            (self.c[i0:i1, j0:j1][max(0, s[0] - rl):s[0] + rl + 1, max(0, s[1] - rl):s[1] + rl + 1] == OBSTACULO)
        prohibido[s] = False
        if prohibido[g]:
            libres = np.argwhere(~prohibido)
            if len(libres) == 0:
                return None
            dd = np.hypot(libres[:, 0] - g[0], libres[:, 1] - g[1])
            k = int(np.argmin(dd))
            if dd[k] * self.res <= 0.6:
                g = (int(libres[k, 0]), int(libres[k, 1]))
            elif not lo_mas_cerca:
                return None
        vecinos = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                   (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        INF = float("inf")
        costo = np.full((H, W), INF)
        padre = np.full((H, W, 2), -1, np.int32)
        costo[s] = 0.0
        abierta = [(math.hypot(g[0] - s[0], g[1] - s[1]), 0.0, s)]
        while abierta:
            _, c, u = heapq.heappop(abierta)
            if u == g:
                break
            if c > costo[u]:
                continue
            for di, dj, w in vecinos:
                v = (u[0] + di, u[1] + dj)
                if not (0 <= v[0] < H and 0 <= v[1] < W) or prohibido[v]:
                    continue
                nc = c + w * (1.0 + extra[v])
                if nc < costo[v]:
                    costo[v] = nc
                    padre[v] = u
                    heapq.heappush(abierta, (nc + math.hypot(g[0] - v[0], g[1] - v[1]), nc, v))
        if costo[g] == INF:
            if not lo_mas_cerca:
                return None
            # la celda alcanzada mas cerca de la meta (empate: la mas barata)
            # puntaje = distancia que falta + 0.15 x lo que cuesta llegar: no da una vuelta de 14 m para
            # quedar 0.6 m mas cerca (26/10: saltaba entre dos destinos y el perro giraba)
            alc = np.argwhere(costo < INF)
            dm = np.hypot(alc[:, 0] - g[0], alc[:, 1] - g[1])
            punt = dm + 0.15 * costo[alc[:, 0], alc[:, 1]]
            k = int(np.argmin(punt))
            g = (int(alc[k, 0]), int(alc[k, 1]))
            meta = self.centro(g[0] + i0, g[1] + j0)
        camino = []
        u = g
        while u != s:
            camino.append(self.centro(u[0] + i0, u[1] + j0))
            u = (int(padre[u][0]), int(padre[u][1]))
        camino.append(tuple(inicio[:2]))
        camino.reverse()
        if tuple(meta[:2]) != camino[-1] and not prohibido[self.celda(*meta)[0] - i0, self.celda(*meta)[1] - j0]:
            camino[-1] = tuple(meta[:2])
        return camino


def punto_adelante(camino, adelante=1.2):
    """El punto del camino a 'adelante' m (sobre el camino) del inicio: el way_point para el planificador
    local (que esquiva solo a 100 Hz). Si el camino es mas corto, la meta."""
    if not camino:
        return None
    hecho = 0.0
    for a, b in zip(camino, camino[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if hecho + L >= adelante and L > 0:
            f = (adelante - hecho) / L
            return (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
        hecho += L
    return camino[-1]


def largo(camino):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(camino, camino[1:]))

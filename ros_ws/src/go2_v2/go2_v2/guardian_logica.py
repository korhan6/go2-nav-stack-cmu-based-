"""Logica del guardian (sin ROS: se prueba sola, v2/pruebas/prueba_guardian.py).

El guardian es el UNICO que le habla al perro (/api/sport/request). Todo lo demas (el planificador
local de CMU, el control PS4, 'go2v2 ...') pasa por aca:

  /cmd_vel (TwistStamped, pathFollower de CMU, ~100 Hz)
      -> recorte a los limites
      -> Move {"x","y","z"} a 50 Hz mientras la orden este fresca
      -> si la orden deja de llegar (nodo caido, red, lo que sea) por mas de 'vigencia' s:
         StopMove YA (el perro NO sigue caminando con la ultima orden)
  Estados que cortan todo: ACOSTADO (echado: un Move lo levantaria), BLOQUEADO (de pie trabado),
  PAUSA (freno de emergencia con CIRCULO), y mientras se para / se echa (secuencia de postura).
  Ademas para si la ODOMETRIA (Point-LIO) o el MAPA DE TERRENO dejan de llegar: el pathFollower de
  CMU sigue publicando la ultima velocidad aunque Point-LIO se caiga (caminaria a ciegas). En modo
  MANUAL (R2 sostenido) esas dos revisiones no aplican: es la salida de emergencia de CMU.

Botones del PS4 que YA usa CMU (no tocar): L1 = borrar el grafo de rutas (FAR), R1 = borrar el mapa
de terreno, L2 = modo autonomo, R2 = modo manual. Por eso las combinaciones van con SHARE.

Cada paso devuelve la lista de pedidos (api_id, parametros|None) a mandar.
"""
import math

import numpy as np

API_DAMP = 1001
API_BALANCESTAND = 1002
API_STOPMOVE = 1003
API_STANDUP = 1004              # "lock stand": de pie con las juntas trabadas
API_STANDDOWN = 1005            # echarse
API_RECOVERYSTAND = 1006
API_MOVE = 1008
API_FREEWALK = 2045             # firmware >= 1.1.7: marcha agil (la usaba la v1 al levantarse)

# control PS4 (driver hid-playstation de Linux, 13 botones)
BOTON = {"cruz": 0, "circulo": 1, "triangulo": 2, "cuadrado": 3, "l1": 4, "r1": 5,
         "l2": 6, "r2": 7, "share": 8, "options": 9, "ps": 10}
EJE_R2 = 5                      # gatillo R2: < -0.1 = sostenido (modo manual de CMU)


def recortar(v, lim):
    return max(-lim, min(lim, v))


class Guardian(object):
    def __init__(self, vx_max=1.0, vy_max=0.6, wz_max=1.5, vigencia=0.25, periodo=0.02,
                 freewalk=True, zona_muerta=0.01, v_min_caminar=0.0, odom_max=0.6, terreno_max=1.5,
                 lidar_max=0.0, giro_max=0.0, salto_vel=0.0, salto_giro=0.0, vueltas_rad=0.0,
                 vueltas_s=8.0, sensor_x=0.3, coherencia_m=0.0, suavizado=None, llegada=None, freno=None):
        self.vx_max, self.vy_max, self.wz_max = vx_max, vy_max, wz_max
        self.vigencia = vigencia
        self.periodo = periodo
        self.freewalk = freewalk
        self.zona_muerta = zona_muerta
        # el Go2 casi no avanza por debajo de ~0.2 m/s (pisa en el sitio). 0 = no tocar la orden
        self.v_min_caminar = v_min_caminar
        self.odom_max = odom_max            # s: edad maxima de la odometria con la que se calculo la orden
        self.terreno_max = terreno_max      # s: sin mapa de terreno nuevo -> la evasion no ve nada
        # s: sin nube nueva del lidar (/registered_scan) -> Point-LIO sigue publicando pose SOLO con la
        # IMU y se va corriendo por el mapa; el perro caminaria a ciegas. 0 = no revisar
        self.lidar_max = lidar_max
        # s girando en el sitio sin avanzar (fuera del modo manual) -> quieto. Es el sintoma de que el
        # planificador no ve NINGUN camino libre (paredes falsas pegadas al perro): sin esto gira para
        # siempre. 0 = no revisar
        self.giro_max = giro_max
        self.t_giro = None
        # salto del SLAM: si la pose de Point-LIO se mueve mas rapido de lo que el perro puede caminar
        # (salto_vel m/s) o girar (salto_giro rad/s), el SLAM perdio el rastro y "se teletransporto": el
        # mapa queda doble y todo lo que planifique esta mal. Queda frenado hasta relanzar. 0 = no revisar
        self.salto_vel = salto_vel
        self.salto_giro = salto_giro
        self.salto = None
        self.pose_prev = None
        # acostado (o caido) el L1 queda a centimetros del piso mirando al suelo: Point-LIO se pierde
        # (26/10: mapa doble y nube "explotada" con el perro acostado). Despues de acostarse o caerse no
        # camina en modo autonomo hasta reiniciar el SLAM ('go2v2 slam-reset', parado y quieto)
        self.slam_dudoso = False
        # da vueltas sin avanzar: en 'vueltas_s' s giro mas de 'vueltas_rad' (sumando a un lado y al otro)
        # y el cuerpo no se alejo mas de 0.5 m -> quieto hasta una orden nueva (meta, teclado, PS4).
        # 26/10: se quedo 30 s girando +60/-60 grados/s frente a la puerta. 0 = no revisar
        self.vueltas_rad = vueltas_rad
        self.vueltas_s = vueltas_s
        self.sensor_x = sensor_x
        self.hist = []
        self.vueltas = None
        # coherencia SLAM vs perro (ver _coherencia): si girando en el sitio el centro de giro no queda al
        # menos 'coherencia_m' m detras del lidar (mediana de 15 giros), el SLAM esta corrido -> quieto
        # hasta slam-reset. 0 = no revisar
        self.coherencia_m = coherencia_m
        # SUAVIZADO de las ordenes (26/10: giraba brusco y los motores cambiaban de sentido de golpe, "tras
        # tras"): cada eje sube/baja con aceleracion limitada (m/s^2, rad/s^2), frenar es mas rapido que
        # arrancar, un cambio de sentido pasa por cero de a poco, y el giro ademas va por un filtro pasa-bajos
        # (tau s) que se come el temblor de la orden de giro. Las paradas de SEGURIDAD siguen instantaneas.
        # None = sin suavizado. dict: acc_lin, dec_lin, acc_lat, dec_lat, acc_giro, dec_giro, tau_giro
        self.suavizado = suavizado
        self.suave = (0.0, 0.0, 0.0)
        # FRENO DE CERCANIA (26/10: a veces se iba de frente contra una persona): ultima red de seguridad,
        # aparte del planificador. Con los obstaculos del mapa de terreno en el marco del cuerpo: si hay algo
        # a menos de 'margen' (+ 'margen_v' x velocidad) de la nariz, no avanza; lo mismo atras y a los
        # costados (de lado). Girar y alejarse si se puede. No aplica en manual (R2 / PS4 sin evasion).
        # None = sin freno. dict: largo, ancho (cuerpo), margen, margen_v
        self.freno = freno
        self.obs_cuerpo = None
        self.t_obs = -1e9
        self.freno_activo = None
        # LLEGADA (26/10: navegaba bien pero al llegar daba circulos, giraba a un lado y al otro y la pose se
        # corria en RViz): cerca de la meta la direccion al punto cambia con cada cm, el seguidor de CMU
        # manda ordenes chiquitas a un lado y al otro y el perro pisa en el sitio. Desde 'frenar_desde' m
        # baja el tope de velocidad y giro en rampa hasta 'v_llegada' / 'giro_llegada' a 'd_llegada' m, y
        # cuando go2_metas dice que llego se queda QUIETO (rampa a cero) hasta otra meta o las palancas.
        # None = sin esto. dict: frenar_desde, d_llegada, v_lejos, v_llegada, giro_llegada
        self.llegada = llegada
        self.meta_dist = None
        self.meta_llego = False
        self.t_meta = -1e9
        self.t_palanca = -1e9           # ultima vez que alguien movio las palancas (PS4 / teclado)
        self.en_meta_avisado = False
        self.envios = []            # [(t, vx, vy, wz enviados)]
        self.poses = []             # [(t, x, y, yaw)]
        self.centros = []           # distancia lidar -> centro de giro, una por giro en el sitio
        self.incoherente = None
        self.edad_odom = None
        self.t_terreno = None
        self.t_lidar = None
        self.manual = False                 # R2 sostenido: modo manual de CMU (sin evasion)
        self.quieto_hasta = -1e9            # durante una secuencia de postura no se camina
        self.motivo_parada = None           # por que esta parado (odometria / terreno), para avisar una vez
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.acostado = False
        self.bloqueado = False
        self.pausa = False
        self.moviendo = False          # la ultima orden mandada fue un Move distinto de cero
        self.t_stop = -1e9
        self.stops_pendientes = 0
        self.agenda = []                # [(t, api, parametros)]: secuencias (levantar, acostar...)
        self.botones_prev = []
        self.avisos = []                # textos para el log
        self.n_move = 0
        self.n_stop = 0
        self.n_vencidas = 0

    # ------------------------------------------------------------------ entradas
    def orden(self, t, vx, vy, wz, edad_odom=None):
        """edad_odom: s desde que se midio la odometria con la que el planificador calculo esta
        orden (el pathFollower sella /cmd_vel con la hora de la odometria)."""
        self.cmd = (vx, vy, wz)
        self.t_cmd = t
        self.edad_odom = edad_odom
        girando = abs(wz) > 0.3 and (vx * vx + vy * vy) ** 0.5 < 0.05 and not self.manual
        if not girando:
            self.t_giro = None
        elif self.t_giro is None:
            self.t_giro = t

    def terreno(self, t):
        self.t_terreno = t

    def lidar(self, t):
        self.t_lidar = t

    def slam_reiniciado(self, t):
        """Point-LIO arranco de nuevo: se olvidan el salto, la pose anterior y la duda."""
        self.salto = None
        self.pose_prev = None
        self.slam_dudoso = False
        self.orden_usuario()
        self.avisos.append("SLAM reiniciado: el mapa empieza de cero aca (los puntos guardados ya no valen)")
        return []

    def pose(self, t, x, y, yaw):
        """Pose de Point-LIO (llamar ~10 Hz). Detecta saltos del SLAM."""
        if self.pose_prev is not None and self.salto is None:
            t0, x0, y0, yaw0 = self.pose_prev
            dt = t - t0
            if dt >= 0.05:
                d = math.hypot(x - x0, y - y0)
                dyaw = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
                if (self.salto_vel > 0 and d > 0.3 and d / dt > self.salto_vel) or \
                        (self.salto_giro > 0 and dyaw > 0.5 and dyaw / dt > self.salto_giro):
                    self.salto = ("el SLAM salto %.2f m y %.0f grados en %.2f s (perdio el rastro: el mapa "
                                  "quedo doble). Relanza: go2v2 detener; go2v2 lanzar" % (d, math.degrees(dyaw), dt))
                    self.avisos.append("ERROR: " + self.salto)
        if self.pose_prev is None or t - self.pose_prev[0] >= 0.05:
            self.pose_prev = (t, x, y, yaw)
        if self.coherencia_m > 0 and self.incoherente is None and not self.manual:
            self._coherencia(t, x, y, yaw)
        if self.vueltas_rad > 0 and not self.manual and self.vueltas is None:
            cx, cy = x - self.sensor_x * math.cos(yaw), y - self.sensor_x * math.sin(yaw)   # centro del cuerpo
            self.hist.append((t, cx, cy, yaw))
            self.hist = [h for h in self.hist if t - h[0] <= self.vueltas_s]
            if self.moviendo and t - self.hist[0][0] >= self.vueltas_s * 0.75:
                giro = sum(abs(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])))
                           for a, b in zip(self.hist, self.hist[1:]))
                lejos = max(math.hypot(h[1] - self.hist[0][1], h[2] - self.hist[0][2]) for h in self.hist)
                if giro > self.vueltas_rad and lejos < 0.5:
                    self.vueltas = ("da vueltas sin avanzar (giro %.0f grados en %.0f s, se movio %.2f m): no "
                                    "encuentra camino. Quieto hasta una orden nueva (otra meta, teclado o PS4)"
                                    % (math.degrees(giro), t - self.hist[0][0], lejos))
                    self.avisos.append("PARO: " + self.vueltas)

    def _coherencia(self, t, x, y, yaw):
        """Girando en el sitio, el lidar (adelante en la cabeza) describe un circulo alrededor del centro del
        cuerpo: con el SLAM sano el centro de giro queda ~0.2-0.3 m DETRAS del lidar (26/10 14:13: 0.20 m).
        Con el SLAM roto queda adelante o en cualquier lado (14:28: -0.21 m, y el perro daba vueltas)."""
        self.poses = [p for p in self.poses if t - p[0] <= 0.35] + [(t, x, y, yaw)]
        self.envios = [e for e in self.envios if t - e[0] <= 0.35]
        p0 = self.poses[0]
        if t - p0[0] < 0.25 or len(self.envios) < 5:
            return
        wz = sum(e[3] for e in self.envios) / len(self.envios)
        quieto = all(abs(e[1]) < 0.05 and abs(e[2]) < 0.05 for e in self.envios)
        dyaw = math.atan2(math.sin(yaw - p0[3]), math.cos(yaw - p0[3]))
        if not quieto or abs(wz) < 0.5 or abs(dyaw) < 0.1:
            return
        h = p0[3] + dyaw / 2
        lateral = -(x - p0[1]) * math.sin(h) + (y - p0[2]) * math.cos(h)
        self.centros.append(lateral / dyaw)
        self.centros = self.centros[-20:]
        self.poses = [self.poses[-1]]                 # muestras sin solaparse
        if len(self.centros) >= 15:
            med = sorted(self.centros)[len(self.centros) // 2]
            if med < self.coherencia_m:
                self.incoherente = ("el SLAM no coincide con el perro: girando en el sitio, el lidar parece "
                                    "estar %.2f m %s del centro de giro (de verdad esta ~0.3 m adelante). El "
                                    "mapa quedo corrido: go2v2 slam-reset (perro parado y quieto)"
                                    % (abs(med), "adelante" if med > 0 else "ATRAS"))
                self.avisos.append("PARO: " + self.incoherente)

    def meta(self, t, distancia, llego):
        """De go2_metas (~5 Hz): distancia a la meta (None = sin meta) y si ya llego."""
        self.meta_dist = distancia
        self.meta_llego = bool(llego) and distancia is not None
        self.t_meta = t
        if not self.meta_llego:
            self.en_meta_avisado = False

    def palanca(self, t):
        """Alguien esta manejando (sticks del PS4 o teclado): la llegada no se mete."""
        self.t_palanca = t

    def _llegando(self, t):
        """Distancia a la meta si hay que aplicar la llegada suave, si no None."""
        if self.llegada is None or self.manual or self.meta_dist is None:
            return None
        if t - self.t_meta > 1.0 or t - self.t_palanca < 1.0:
            return None
        return self.meta_dist

    def _frenar_llegada(self, t, vx, vy, wz):
        d = self._llegando(t)
        if d is None:
            return vx, vy, wz
        if self.meta_llego:
            if not self.en_meta_avisado:
                self.en_meta_avisado = True
                self.avisos.append("llego a la meta (a %.2f m): quieto hasta otra meta" % d)
            return 0.0, 0.0, 0.0
        L = self.llegada
        if d >= L["frenar_desde"]:
            return vx, vy, wz
        f = max(0.0, min(1.0, (d - L["d_llegada"]) / max(L["frenar_desde"] - L["d_llegada"], 1e-3)))
        vtope = L["v_llegada"] + f * (L["v_lejos"] - L["v_llegada"])
        wtope = L["giro_llegada"] + f * (self.wz_max - L["giro_llegada"])
        v = (vx * vx + vy * vy) ** 0.5
        if v > vtope:
            vx, vy = vx * vtope / v, vy * vtope / v
        return vx, vy, recortar(wz, wtope)

    def obstaculos(self, t, xy):
        """Obstaculos cercanos (n, 2) en el marco del CENTRO del cuerpo (x adelante, y izquierda)."""
        self.obs_cuerpo = np.asarray(xy, np.float64).reshape(-1, 2)
        self.t_obs = t

    def _frenar_cerca(self, t, vx, vy):
        F = self.freno
        if F is None or self.manual or self.obs_cuerpo is None or t - self.t_obs > 1.0:
            self.freno_activo = None
            return vx, vy
        o = self.obs_cuerpo
        x, y = o[:, 0], o[:, 1]
        hl, hw = F["largo"] / 2, F["ancho"] / 2
        mx = F["margen"] + F["margen_v"] * abs(vx)
        my = 0.7 * F["margen"] + F["margen_v"] * abs(vy)
        motivo = None
        if vx > 0 and np.any((x > hl) & (x < hl + mx) & (np.abs(y) < hw)):
            vx, motivo = 0.0, "adelante"
        elif vx < 0 and np.any((x < -hl) & (x > -hl - mx) & (np.abs(y) < hw)):
            vx, motivo = 0.0, "atras"
        if vy > 0 and np.any((y > hw) & (y < hw + my) & (np.abs(x) < hl)):
            vy, motivo = 0.0, motivo or "a la izquierda"
        elif vy < 0 and np.any((y < -hw) & (y > -hw - my) & (np.abs(x) < hl)):
            vy, motivo = 0.0, motivo or "a la derecha"
        if motivo and motivo != self.freno_activo:
            self.avisos.append("FRENO: algo a menos de %.0f cm %s del perro (el planificador queria ir para alla)"
                               % (100 * (mx if motivo in ("adelante", "atras") else my), motivo))
        self.freno_activo = motivo
        return vx, vy

    def orden_usuario(self):
        """Meta nueva, teclado o PS4: se olvida el 'da vueltas' y el 'no avanza'."""
        self.vueltas = None
        self.hist = []
        self.incoherente = None
        self.envios = []
        self.poses = []
        self.centros = []

    def modo_manual(self, ejes):
        """/joy de 'go2v2 manejar': solo el gatillo R2 (manual sin evasion), sin combinaciones."""
        self.manual = len(ejes) > EJE_R2 and ejes[EJE_R2] < -0.1

    def _suavizar(self, vx, vy, wz):
        s = self.suavizado
        dt = self.periodo
        px, py, pw = self.suave

        def lim(p, objetivo, sube, baja):
            # mas rapido en el mismo sentido -> 'sube'; frenar o cambiar de sentido -> 'baja' (pasa por 0)
            a = sube if (abs(objetivo) > abs(p) and objetivo * p >= 0) else baja
            d = objetivo - p
            return p + max(-a * dt, min(a * dt, d))
        nx = lim(px, vx, s["acc_lin"], s["dec_lin"])
        ny = lim(py, vy, s["acc_lat"], s["dec_lat"])
        # filtro solo mientras gira (para frenar no: el filtro haria la parada lenta)
        wf = wz if wz == 0.0 else pw + (wz - pw) * min(1.0, dt / max(s["tau_giro"], 1e-3))
        nw = lim(pw, wf, s["acc_giro"], s["dec_giro"])
        self.suave = (nx, ny, nw)
        return nx, ny, nw

    def _cortar(self, t):
        """Deja de caminar YA: StopMove ahora y otro un poco despues (por si se pierde uno)."""
        self.suave = (0.0, 0.0, 0.0)
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.moviendo = False
        self.stops_pendientes = 1          # uno de respaldo 0.15 s despues
        self.t_stop = t
        return [(API_STOPMOVE, None)]

    def _soltar(self):
        """Olvida la orden en curso sin mandar nada (antes de pararse: un StopMove a mitad de un
        StandUp podria cortarlo)."""
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self.moviendo = False
        self.stops_pendientes = 0

    def acostar(self, t):
        self.acostado = True
        self.slam_dudoso = True
        self.agenda = [(t + 0.5, API_STANDDOWN, None)]
        self.quieto_hasta = t + 2.0
        self.avisos.append("ACOSTADO: no camina hasta 'levantar' (L1+TRIANGULO o go2v2 levantar)")
        return self._cortar(t)

    def levantar(self, t):
        self.acostado = False
        self.bloqueado = False
        self.pausa = False
        self.agenda = [(t, API_STANDUP, None), (t + 1.5, API_BALANCESTAND, None)]
        if self.freewalk:
            self.agenda.append((t + 2.5, API_FREEWALK, None))
        # que no arranque a caminar a medio pararse (el planificador puede tener una orden lista)
        self.quieto_hasta = t + (3.0 if self.freewalk else 2.0)
        self.avisos.append("levantando")
        self._soltar()
        return []

    def recuperar(self, t):
        """Se cayo o quedo raro: RecoveryStand (se para desde cualquier posicion) y equilibrio."""
        self.slam_dudoso = True
        self.acostado = False
        self.bloqueado = False
        self.agenda = [(t, API_RECOVERYSTAND, None), (t + 2.5, API_BALANCESTAND, None)]
        if self.freewalk:
            self.agenda.append((t + 3.5, API_FREEWALK, None))
        self.quieto_hasta = t + (4.0 if self.freewalk else 3.0)
        self.avisos.append("recuperando (RecoveryStand)")
        self._soltar()
        return []

    def bloquear(self, t):
        self.bloqueado = True
        self.agenda = [] if self.acostado else [(t + 0.5, API_STANDUP, None)]
        self.avisos.append("BLOQUEADO: quieto y trabado (L1+SHARE o go2v2 desbloquear)")
        return self._cortar(t)

    def desbloquear(self, t):
        if not self.bloqueado:
            return []
        self.bloqueado = False
        if not self.acostado:
            self.agenda = [(t, API_BALANCESTAND, None)]
            if self.freewalk:
                self.agenda.append((t + 1.0, API_FREEWALK, None))
            self.quieto_hasta = t + 1.5
        self.avisos.append("desbloqueado")
        return []

    def alternar_pausa(self, t):
        self.pausa = not self.pausa
        self.avisos.append("PAUSA: freno de emergencia (CIRCULO otra vez para seguir)" if self.pausa
                           else "pausa terminada: sigo")
        return self._cortar(t) if self.pausa else []

    def joystick(self, t, botones, ejes=()):
        """Botones del PS4 (flanco de subida; L1 y R1 son de CMU):
        CRUZ = acostar, TRIANGULO = levantar, CIRCULO = pausa (freno de emergencia),
        SHARE + CUADRADO = bloquear/desbloquear, SHARE + CIRCULO = recuperar (RecoveryStand).
        (SHARE + CRUZ / SHARE + TRIANGULO tambien acuestan / levantan, como antes)"""
        prev = self.botones_prev
        self.botones_prev = list(botones)
        self.manual = len(ejes) > EJE_R2 and ejes[EJE_R2] < -0.1

        def apretado(nombre):
            i = BOTON[nombre]
            return i < len(botones) and botones[i] and not (i < len(prev) and prev[i])

        def sostenido(nombre):
            i = BOTON[nombre]
            return i < len(botones) and bool(botones[i])
        out = []
        if apretado("cruz"):
            out += self.acostar(t)
        elif apretado("triangulo"):
            out += self.levantar(t)
        elif sostenido("share"):
            if apretado("cuadrado"):
                out += self.desbloquear(t) if self.bloqueado else self.bloquear(t)
            elif apretado("circulo"):
                out += self.recuperar(t)
        elif apretado("circulo"):
            out += self.alternar_pausa(t)
        return out

    def _sin_datos(self, t):
        """Por que no se puede caminar ahora (None = se puede)."""
        if self.salto is not None:
            return self.salto
        if self.manual:
            return None
        if self.vueltas is not None:
            return self.vueltas
        if self.incoherente is not None:
            return self.incoherente
        if self.slam_dudoso:
            return ("el perro se acosto o se cayo: acostado el SLAM se pierde. Con el perro PARADO y quieto: "
                    "go2v2 slam-reset")
        if self.edad_odom is not None and self.edad_odom > self.odom_max:
            return "la odometria (Point-LIO) tiene %.1f s de atraso" % self.edad_odom
        if self.lidar_max > 0:
            if self.t_lidar is None:
                return "todavia no llega la nube del lidar (Point-LIO iria a ciegas)"
            if t - self.t_lidar > self.lidar_max:
                return ("no llega la nube del lidar hace %.1f s (Point-LIO va solo con la IMU y se corre)"
                        % (t - self.t_lidar))
        if self.giro_max > 0 and self.t_giro is not None and t - self.t_giro > self.giro_max:
            return ("gira en el sitio hace %.0f s sin avanzar: el planificador no ve camino libre "
                    "(mira TerrainMap en RViz; R borra el mapa de terreno)" % (t - self.t_giro))
        if self.t_terreno is None:
            return "todavia no llega el mapa de terreno (la evasion no ve nada)"
        if t - self.t_terreno > self.terreno_max:
            return "no llega el mapa de terreno hace %.1f s (la evasion no ve)" % (t - self.t_terreno)
        return None

    # ------------------------------------------------------------------ salida
    def paso(self, t):
        """Lo que hay que mandarle al perro ahora (llamar cada 'periodo' s)."""
        out = []
        while self.agenda and self.agenda[0][0] <= t:
            _, api, par = self.agenda.pop(0)
            out.append((api, par))
        if self.stops_pendientes and t - self.t_stop >= 0.15:
            out.append((API_STOPMOVE, None))
            self.stops_pendientes -= 1
            self.t_stop = t
            self.n_stop += 1
        if self.acostado or self.bloqueado or self.pausa or t < self.quieto_hasta:
            return out
        fresca = t - self.t_cmd <= self.vigencia
        if not fresca:
            if self.moviendo:
                # la orden se vencio en pleno movimiento: parar YA
                self.n_vencidas += 1
                self.avisos.append("sin ordenes del planificador hace %.2f s: StopMove" % (t - self.t_cmd))
                out += self._cortar(t)
                self.n_stop += 1
            return out
        motivo = self._sin_datos(t)
        if motivo is not None:
            if self.motivo_parada is None:
                self.avisos.append("PARO: " + motivo)       # una vez, no 50 por segundo
            self.motivo_parada = motivo
            if self.moviendo:
                out += self._cortar(t)
                self.n_stop += 1
            return out
        if self.motivo_parada is not None:
            self.avisos.append("datos al dia otra vez: sigo")
            self.motivo_parada = None
        # avance y lateral se escalan JUNTOS: recortarlos por separado le cambiaria la direccion a la
        # diagonal con la que el planificador esquiva (y podria rozar lo que estaba evitando)
        vx, vy = self.cmd[0], self.cmd[1]
        k = 1.0
        if abs(vx) > self.vx_max:
            k = min(k, self.vx_max / abs(vx))
        if abs(vy) > self.vy_max:
            k = min(k, self.vy_max / abs(vy))
        vx, vy = vx * k, vy * k
        wz = recortar(self.cmd[2], self.wz_max)
        vx, vy, wz = self._frenar_llegada(t, vx, vy, wz)
        vx, vy = self._frenar_cerca(t, vx, vy)
        z = self.zona_muerta
        objetivo_cero = abs(vx) < z and abs(vy) < z and abs(wz) < z
        if self.suavizado is not None and (self.moviendo or not objetivo_cero):
            vx, vy, wz = self._suavizar(0.0 if objetivo_cero else vx, 0.0 if objetivo_cero else vy,
                                        0.0 if objetivo_cero else wz)
        if abs(vx) < z and abs(vy) < z and abs(wz) < z:
            if self.moviendo:
                out += self._cortar(t)
                self.n_stop += 1
            return out
        if self.v_min_caminar > 0:
            v = (vx * vx + vy * vy) ** 0.5
            if 0.05 < v < self.v_min_caminar:
                k = self.v_min_caminar / v
                vx, vy = vx * k, vy * k
        self.moviendo = True
        self.stops_pendientes = 0
        self.n_move += 1
        out.append((API_MOVE, {"x": round(vx, 4), "y": round(vy, 4), "z": round(wz, 4)}))
        if self.coherencia_m > 0:
            self.envios.append((t, vx, vy, wz))
        return out

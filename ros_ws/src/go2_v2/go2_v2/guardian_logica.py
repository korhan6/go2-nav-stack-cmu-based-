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
                 vueltas_s=8.0, sensor_x=0.3, coherencia_m=0.0):
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

    def _cortar(self, t):
        """Deja de caminar YA: StopMove ahora y otro un poco despues (por si se pierde uno)."""
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
        """Combinaciones del PS4 (flanco de subida), con SHARE sostenido (L1 y R1 son de CMU):
        SHARE + CRUZ = acostar, SHARE + TRIANGULO = levantar, SHARE + CUADRADO = bloquear/desbloquear,
        SHARE + CIRCULO = recuperar (RecoveryStand). CIRCULO solo = pausa (freno de emergencia)."""
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
        if sostenido("share"):
            if apretado("cruz"):
                out += self.acostar(t)
            elif apretado("triangulo"):
                out += self.levantar(t)
            elif apretado("cuadrado"):
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
        z = self.zona_muerta
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

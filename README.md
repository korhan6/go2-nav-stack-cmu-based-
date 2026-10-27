# Go2 v2 — stack de CMU en Docker

La v2 cambia el corazón de la navegación por el **autonomy stack de CMU para el Go2**
([jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)). Es el de los
videos donde alguien se le cruza al perro y este lo esquiva en diagonal sin detenerse.

| | v1 (Nav2 propio) | v2 (CMU) |
|---|---|---|
| Odometría | patas × 1.839 (escala corregida a mano) | **Point-LIO** con la IMU del L1 (lidar + inercial) |
| Evasión | MPPI a 15 Hz + costmaps + collision_monitor | **planificador local con cientos de trayectorias precalculadas, a ~100 Hz**: elige otra en el acto, sin replanificar |
| Rutas largas | SmacPlanner2D + BT de Nav2 | **FAR planner** (grafo de visibilidad que se arma mientras camina) |
| Movimiento | gira y avanza | avanza, gira **y se desliza de lado** cerca de los obstáculos (omnidireccional) |
| Le habla al perro | driver (Python) | **go2_guardian**: Move a 50 Hz y StopMove si deja de recibir órdenes por 0.25 s |
| ROS | Jazzy, en tu PC | **Humble, dentro de Docker** (igual que en la Jetson de la mochila) |

La v1 queda intacta: sigue en `~/go2nav_ws` (comando `go2`) y en la rama `claude/v1-checkpoint` del
repo. **No corras las dos a la vez**: las dos le mandarían órdenes al perro, así que `go2v2` se niega
a arrancar si ve la v1 corriendo.

## Qué agrega go2_v2 encima de CMU

- **go2_sensores**: hace la misma cuenta que el `transform_everything.py` de CMU (probada número
  por número contra la original en `pruebas/prueba_transformacion.py`), pero vectorizada. Tarda
  ~1–2 ms por nube; la original iba punto por punto en Python y es la "demora de más de 1 s" que
  CMU reporta en laptops. Además, la IMU no arranca hasta conocer el desfase de relojes.
- **go2_guardian**: es el único nodo que le habla al perro. Hace esto:
  - Pone límites duros de velocidad.
  - Tiene un watchdog: si el planificador se cae, StopMove en 0.25 s.
  - **Frena si Point-LIO o el mapa de terreno dejan de llegar.** El seguidor de CMU sigue mandando
    la última velocidad aunque Point-LIO se caiga, y el perro caminaría a ciegas.
  - No camina mientras se está parando o echando.
  - Maneja los estados acostar, levantar, bloquear, pausa y recuperar, desde el PS4 o por comando.
  - Apaga la evasión de obstáculos de Unitree, que frenaba al perro.
  - Manda un StopMove final al cerrarse con Ctrl+C.
- **Comandos** `go2v2 ir / guardar / puntos / parar / estado`.
- **Pruebas sin ROS** en `pruebas/`: la matemática contra la original de CMU, la lógica del
  guardián, y los tres nodos corriendo con un ROS simulado.

## Instalar (una vez)

```bash
cd ~/go2v2                 # esta carpeta
./go2v2 instalar           # construye la imagen (15-25 min la primera vez) y deja el comando 'go2v2'
go2v2 red                  # buffer de red grande para las nubes del L1 (pide sudo)
go2v2 calibrar             # IMU del L1: perro DE PIE y con 1 m libre (pasitos, quieto, gira 20 s)
```

Si `docker` pide sudo: `sudo usermod -aG docker $USER` y cierra sesión (o reinicia).

## Usar

```bash
go2v2 lanzar               # SLAM + evasión + rutas + RViz
go2v2 lanzar velocidad:=0.6 ruta:=false
```

En RViz, el botón **Goalpoint** manda al perro a donde hagas clic. Si alguien se cruza, lo esquiva.
También desde otra terminal:

```bash
go2v2 ir 3 1.5             # metros desde donde arrancó el perro (x adelante, y izquierda)
go2v2 guardar cocina ; go2v2 ir cocina
go2v2 parar                # deja de navegar
go2v2 pausa                # freno de emergencia (otra vez para seguir)
go2v2 estado               # demora, Hz, pose, estado del guardián
```

**Control PS4** (por defecto SIN evasión: manejas tú; OPTIONS cambia a CON evasión):

| Botón | Qué hace |
|---|---|
| Stick izquierdo | Moverse: adelante / atrás / derecha / izquierda (de lado) |
| Stick derecho | Girar |
| CRUZ | Acostar |
| TRIÁNGULO | Levantar |
| CÍRCULO | Pausa (freno de emergencia; otra vez para seguir) |
| OPTIONS | Sin evasión ↔ con evasión (mismos sticks; CMU esquiva solo) |
| L2 sostenido | Modo autónomo (sigue la meta) |
| R1 | Borra el mapa de terreno |
| L1 | Borra el grafo de rutas de FAR (lo de CMU) |
| SHARE + CUADRADO | Bloquear / desbloquear |
| SHARE + CÍRCULO | Recuperar (RecoveryStand: se cayó o quedó raro) |

**Llegada a la meta:** si la meta le queda de frente llega hasta ella; si queda al costado o atrás se
para a ~30 cm sin girar en el sitio. Desde 1.5 m frena en rampa y al llegar se queda quieto hasta otra
meta (o hasta que muevas el control). Ajustes en `config/guardian.yaml` (`llegada_suave`, `frenar_desde`...).

## Manejar con el teclado y armar el mapa

Con `go2v2 lanzar` corriendo, en otra terminal:

```bash
go2v2 manejar              # WASD: W/Q/E con evasion, A/D girar, S atras, Z/C de lado, ESPACIO frena
go2v2 mapa                 # guarda el mapa armado en ~/.go2_v2/mapas/mapa_FECHA.pcd (o G en el teclado)
go2v2 mapa-borrar          # empezar el mapa de cero
```

Mantener la tecla = camina; soltarla = frena. El mapa se ve en RViz (display **Mapa**, color por altura).
El .pcd se abre con CloudCompare u Open3D. Todavia es solo para verlo: la navegacion no lo recarga.

## Esquivar (26/10: a veces se iba de frente contra una persona)

Tres cosas, ninguna es "subir o bajar la resolucion" del lidar:

- **Pisadas solo con fantasmas confirmados**: antes, por donde el perro ya habia caminado se borraba todo
  lo bajo (< 70 cm) PARA SIEMPRE: si una persona se paraba ahi, sus piernas desaparecian. Ahora solo se
  borra una celda si el cuerpo del perro estuvo encima MIENTRAS el mapa decia que habia algo (eso prueba
  que era un fantasma). Lo que aparece despues (una persona) se ve normal.
- **Zona ciega del lidar 0.5 -> 0.3 m** (`ciego:=0.5` vuelve a CMU): Point-LIO tiraba todo lo que estaba a
  menos de medio metro del L1 (va en la nariz); el mapa recordaba a la persona 2 s y despues el perro
  avanzaba. La caja que borra las propias patas ahora llega a +5 cm delante del L1 (`caja_adelante`).
- **Freno de cercania** en el guardian (config/guardian.yaml, `freno_*`): si hay algo a menos de ~30 cm
  (+40 cm por m/s) de la nariz, la cola o el costado hacia donde se mueve, no se mueve hacia ahi (girar y
  alejarse si). Es aparte del planificador. No aplica manejando sin evasion. `go2v2 estado`: "freno".
- Al girar en el sitio revisa que la cola/cabeza no barra nada (`revisar_giro:=false` vuelve a CMU).

## Fantasmas del lidar

**Calibrar el filtro** (una vez por lugar/perro, queda guardado en `~/.go2_v2/fantasmas.yaml`):

```bash
go2v2 calibrar-fantasmas          # con 'go2v2 lanzar' corriendo: se para, da UNA vuelta despacio (~20 s)
go2v2 lanzar calibrar_fantasmas:=true   # o que lo haga solo al lanzar
```

Separa lo real (aparece en muchas nubes) del ruido (puntos sueltos) y elige `min_puntos` y `ventana`: el
que mas ruido saca guardando >= 97 % de lo real. Deja ~1 m libre alrededor; mejor cerca de paredes y muebles.


El L1 tira puntos sueltos que el planificador toma como obstaculos. Dos reglas:

- **go2_fantasmas**: un punto cerca del perro se queda solo si se repite en las ultimas nubes (0.5 s) en
  el mismo lugar (al menos 5 puntos en su vecindario de 30 cm). Lo real se repite; el ruido no. El
  piso nunca se toca. `go2v2 lanzar filtro:=false` lo apaga; `fantasmas_min_puntos:=7` filtra mas.
- **Paredes por "sin datos"** (la causa de que GIRARA sin parar): CMU pone una pared en cada cuadro de
  20 cm entre 0.3 y 1.8 m delante del perro con menos de 10 puntos. Esa zona va pegada al perro: si
  gira, la pared gira con el y vuelve a quedar delante. Aca solo cuenta un cuadro SIN ningun punto y
  hasta 1 m (`terreno_min_puntos:=10 terreno_sin_datos_hasta:=1.8` vuelve a CMU). Un hueco real o
  un piso 0.6 m mas abajo siguen frenando.
- **Anti-giro** (guardian): si gira en el sitio mas de 6 s sin avanzar, se queda quieto y avisa
  (`parado_por` en `go2v2 estado`). En manual (A/D del teclado, R2) no aplica.

## Pasar por lugares angostos

CMU revisa choques en un radio de 0.55 m x la escala del camino: con su escala minima (0.5) el perro
necesita ~0.55 m libres. Aca baja hasta 0.45 (~0.5 m) en pasos de 0.1, solo si no hay camino mas ancho
(`escala_min:=0.5` = CMU). FAR (rutas largas) usa un perro de 0.45 m en vez de 0.6
(`ancho_ruta:=0.6` = CMU).

Si igual queda basura: R en el teclado o R1 en el PS4 borra el mapa de terreno.

## Metas, SLAM y protecciones (26/10)

- **go2_metas**: metas a menos de 3 m y con la linea recta libre van DIRECTO al planificador local; el
  resto con FAR, y al acercarse pasa a directo para llegar a la meta exacta. Si FAR pasa 4 s sin dar
  ruta (meta en la escalera, otro piso, detras de algo cerrado) el perro se queda quieto y avisa.
  `directo_max:=0` = todo con FAR, como CMU.
- **`go2v2 slam-reset`**: reinicia Point-LIO (y FAR, terreno y mapa) sin relanzar todo. El perro PARADO
  y quieto. El origen pasa a ser donde esta el perro: los puntos guardados dejan de valer.
- El guardian frena (y dice por que en `go2v2 estado` -> `parado_por`) si:
  - deja de llegar la nube del lidar (0.5 s) o el mapa de terreno;
  - el SLAM salta (se mueve mas rapido de lo que el perro puede);
  - el SLAM no coincide con el perro: girando en el sitio, el centro de giro debe quedar detras del
    lidar (sano ~0.2 m). Si no, el mapa quedo corrido -> `go2v2 slam-reset`;
  - el perro se acosto o se cayo: acostado el L1 mira al piso y el SLAM se pierde -> `slam-reset`;
  - gira en el sitio mas de 6 s, o da vueltas 8 s sin alejarse 0.5 m (hasta una orden nueva).
  En manual (teclado A/D/S/Z/C, R2 del PS4) igual se puede mover.
- **`go2v2 diagnostico 60`**: graba lo que decide la navegacion en `~/go2v2/diagnostico` (ya achicado)
  para revisarlo.
- **`go2v2 deriva 20`**: con el perro quieto, cuanto se corre el SLAM.
- Obstaculos desde 20 cm de alto (CMU: 30; el R1 acostado era invisible): `altura_obstaculo:=0.3`.

## Simulador (sin el perro)

Usa el entorno 3D de CMU (Unity) con un robot simulado. Corre la misma navegación y el mismo
guardián, y queda aislado del perro: usa otro dominio DDS, el 42.

1. Baja el entorno de Unity de CMU: [carpeta de CMU en Google Drive](https://drive.google.com/drive/folders/1CeAA5aeXCk8Sh3ZuJR7UME6sQZFuaorf).
2. Descomprímelo para que quede `~/.go2_v2/unity/environment/Model.x86_64`, con `map.ply` y el
   resto al lado.
3. Lánzalo:
   ```bash
   go2v2 sim                  # o: go2v2 sim ruta:=false velocidad:=0.8
   ```

Desde otra terminal funcionan `go2v2 ir / guardar / parar / pausa / estado`. El PS4 y los botones
de RViz también.

| Se prueba en el simulador | No se prueba (necesita el perro) |
|---|---|
| Evasión de obstáculos, rutas con FAR, modos del PS4 | Point-LIO y la nube real del L1 |
| `go2v2 ir / guardar / puntos / parar / estado` | go2_sensores y la calibración de la IMU |
| Guardián: pausa, acostar y bloquear frenan al robot simulado | Las posturas reales (echarse, pararse) |

## Si algo no anda

- **`go2v2 estado`**:
  - `demora_ms` debería ser < 150. Si es mucho mayor, se están perdiendo nubes: corre `go2v2 red`.
  - Si `pose` dice que no llega, Point-LIO no arrancó: revisa que haya nube (`nubes` > 0) y IMU.
  - `parado_por` del guardián dice por qué no camina, por ejemplo "la odometría tiene 1.2 s de
    atraso" o "no llega el mapa de terreno".
- **"el reloj del perro SALTÓ"**: el perro se reinició con la v2 andando. Relánzala con
  `go2v2 detener` y después `go2v2 lanzar`.
- **El perro se va torcido o el mapa se dobla**: recalibra la IMU (`go2v2 calibrar`) y espera
  10–20 s quieto al arrancar (CMU avisa que al inicio puede haber demora).
- **Obstáculos bajos**: el L1 es ruidoso y CMU no ve lo que mide menos de ~30 cm. Para cosas más
  bajas hace falta otro sensor (la mochila con Mid-360 es la mejora oficial).
- **`go2v2 log`** muestra los últimos mensajes.
- **`go2v2 grabar 60`** graba los datos crudos para revisarlos sin el perro.

## Cambiar la v2

El código de go2_v2 (`ros_ws/src/go2_v2`) se monta dentro del Docker: un cambio en Python vale con
solo volver a `go2v2 lanzar`, sin reconstruir. Los parámetros del guardián están en
`ros_ws/src/go2_v2/config/guardian.yaml`.

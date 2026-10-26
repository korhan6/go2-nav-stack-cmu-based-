"""Achica una grabacion de 'go2v2 diagnostico' para mandarsela a Claude (limite ~400 MB).
Uso: python3 ~/go2v2/diagnostico/achicar.py ~/go2v2/diagnostico/nav_XXXX/nav_XXXX_0.db3
Deja nav_XXXX_0_chico.db3 al lado (el original no se toca)."""
import os
import shutil
import sqlite3
import sys

src = os.path.expanduser(sys.argv[1])
dst = src.replace(".db3", "_chico.db3")
shutil.copy(src, dst)
c = sqlite3.connect(dst)
ids = {name: i for i, name in c.execute("select id, name from topics")}
# 1 de cada N mensajes de los topicos pesados (la odometria sale a miles por segundo)
cada = {"/state_estimation": 20, "/terrain_map": 4, "/free_paths": 4}
for nombre, n in cada.items():
    if nombre in ids:
        c.execute("delete from messages where topic_id = ? and (id % ?) != 0", (ids[nombre], n))
c.commit()
c.execute("vacuum")
c.close()
print("listo: %s (%.0f MB)" % (dst, os.path.getsize(dst) / 1e6))

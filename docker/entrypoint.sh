#!/bin/bash
# Entorno de la v2 dentro del Docker: ROS 2 Humble + CMU + go2_v2 + CycloneDDS por el cable del perro.
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

if [ -n "$GO2V2_SIM" ]; then
  # simulacion: otro dominio DDS (42) -> nada de lo que corre aca le llega a un perro real
  export ROS_DOMAIN_ID=42
  unset CYCLONEDDS_URI
  [ -n "$GO2V2_SILENCIO" ] || echo "go2v2: SIMULACION (dominio DDS 42, aislado del perro)"
  exec "$@"
fi

# la interfaz del cable: la que tenga una IP 192.168.123.x (el perro es 192.168.123.161)
if [ -z "$GO2_IF" ]; then
  GO2_IF=$(ip -o -4 addr show 2>/dev/null | awk '$4 ~ /^192\.168\.123\./ {print $2; exit}')
fi
if [ -n "$GO2_IF" ]; then
  # AllowMulticast=spdp: multicast SOLO para descubrir nodos; los datos van por unicast. Sin esto,
  # todas las nubes internas (registered_scan, terrain_map...) salian por el cable y saturaban al perro.
  # buffer de recepcion grande: las nubes del L1 llegan en muchos pedazos UDP (ver 'go2v2 red')
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$GO2_IF\" priority=\"default\" multicast=\"default\"/></Interfaces><AllowMulticast>spdp</AllowMulticast></General><Internal><SocketReceiveBufferSize min=\"default\" max=\"10MB\"/></Internal></Domain></CycloneDDS>"
  [ -n "$GO2V2_SILENCIO" ] || echo "go2v2: perro por $GO2_IF (CycloneDDS)"
else
  [ -n "$GO2V2_SILENCIO" ] || echo "go2v2: !! ninguna interfaz con IP 192.168.123.x: el perro no esta conectado por cable"
fi
exec "$@"

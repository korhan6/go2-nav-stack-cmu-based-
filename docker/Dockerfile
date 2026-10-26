# Go2 v2: stack de CMU (autonomy_stack_go2) + go2_v2, en ROS 2 Humble (Ubuntu 22.04).
# Por que Humble y no Foxy: CycloneDDS 0.10 (la del perro) viene de fabrica en Humble (en Foxy hay
# que compilarla) y Humble tiene soporte hasta 2027. La demora de >1 s que CMU vio en Humble era la
# transformacion de sensores en Python: go2_v2 la reemplaza por una vectorizada (~1-2 ms por nube).
#
#   docker build -t go2v2 -f v2/docker/Dockerfile v2      (lo hace 'go2v2 instalar')
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# llave del repositorio de ROS: si la imagen trae la vieja (vencio en 2025), se instala la nueva
# (ros2-apt-source, el metodo oficial)
RUN if ! apt-get update; then \
      apt-get install -y --no-install-recommends curl ca-certificates; \
      rm -f /etc/apt/sources.list.d/ros2*.list /usr/share/keyrings/ros-archive-keyring.gpg; \
      V=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
          | grep -F '"tag_name"' | awk -F'"' '{print $4}'); \
      curl -fsSL -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.jammy_all.deb"; \
      apt-get install -y /tmp/ros2-apt-source.deb && apt-get update; \
    fi

RUN apt-get update && apt-get install -y --no-install-recommends \
      git python3-pip python3-numpy python3-yaml python-is-python3 python3-colcon-common-extensions \
      libusb-dev libpcl-dev libeigen3-dev libopencv-dev python3-dev libgoogle-glog-dev \
      ros-humble-tf2-eigen ros-humble-rclcpp-components ros-humble-message-filters \
      ros-humble-perception-pcl ros-humble-pcl-ros ros-humble-pcl-conversions \
      ros-humble-sensor-msgs-py ros-humble-tf-transformations ros-humble-tf2-sensor-msgs \
      ros-humble-cv-bridge ros-humble-joy ros-humble-rmw-cyclonedds-cpp \
      ros-humble-rosidl-generator-dds-idl ros-humble-rosbag2-cpp \
      iproute2 iputils-ping joystick \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir transforms3d

# el stack de CMU, en el commit probado (feb 2026). La camara (go2_h264_repub, gstreamer) no hace
# falta para navegar: se saltea
ARG CMU_REPO=https://github.com/jizhang-cmu/autonomy_stack_go2.git
ARG CMU_COMMIT=43d5f54b389b251713f0097893c30fa76c870d54
RUN git clone ${CMU_REPO} /ws && cd /ws && git checkout ${CMU_COMMIT} \
    && touch /ws/src/utilities/unitree_pkgs/go2_h264_repub/COLCON_IGNORE

# dependencias que falten (rosdep); -r: sigue aunque alguna clave no exista
RUN source /opt/ros/humble/setup.bash && apt-get update \
    && (rosdep update --rosdistro humble || true) \
    && (rosdep install --from-paths /ws/src --ignore-src -r -y --rosdistro humble || true) \
    && rm -rf /var/lib/apt/lists/*

# primero CMU (la parte larga: ~10-20 min), despues go2_v2 (asi cambiar go2_v2 no recompila CMU).
# JOBS: compilaciones en paralelo. Point-LIO y PCL usan ~2-3 GB de RAM cada una: con 16 GB, 4 va
# bien (con mas, la laptop se puede quedar sin memoria y el build muere sin explicar por que)
ARG JOBS=4
ENV MAKEFLAGS=-j${JOBS}
RUN source /opt/ros/humble/setup.bash && cd /ws \
    && (colcon build --symlink-install --continue-on-error --parallel-workers ${JOBS} \
        --cmake-args -DCMAKE_BUILD_TYPE=Release > /tmp/build.log 2>&1; \
        grep -E "^(Summary|Failed|Aborted)|packages? (finished|failed|aborted)" /tmp/build.log; true) \
    # lo IMPRESCINDIBLE tiene que estar (si algo accesorio fallo, sigue; si falta esto, se corta
    # aca mostrando el error real)
    && for f in point_lio_unilidar/lib/point_lio_unilidar/pointlio_mapping \
                local_planner/lib/local_planner/localPlanner local_planner/lib/local_planner/pathFollower \
                terrain_analysis/lib/terrain_analysis/terrainAnalysis \
                terrain_analysis_ext/lib/terrain_analysis_ext/terrainAnalysisExt \
                far_planner/lib/far_planner/far_planner graph_decoder/lib/graph_decoder/graph_decoder \
                calibrate_imu/lib/calibrate_imu/calibrate_imu unitree_api/share/unitree_api/package.xml; do \
         if [ ! -e /ws/install/$f ]; then echo "FALTA: $f"; tail -n 120 /tmp/build.log; exit 1; fi; \
       done

# librerias que pide el simulador Unity de CMU ('go2v2 sim'). Va aca y no arriba para no
# recompilar todo CMU en una imagen ya construida
RUN apt-get update && apt-get install -y --no-install-recommends \
      libglu1-mesa libxcursor1 libxrandr2 libxinerama1 libxi6 libvulkan1 libgtk-3-0 mesa-utils \
    && rm -rf /var/lib/apt/lists/*

COPY ros_ws/src/go2_v2 /ws/src/go2_v2
RUN source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && cd /ws \
    && colcon build --symlink-install --packages-select go2_v2

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && mkdir -p /root/Desktop /root/.go2_v2
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=0 \
    SDL_JOYSTICK_DISABLE_UDEV=1 \
    PYTHONDONTWRITEBYTECODE=1
WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

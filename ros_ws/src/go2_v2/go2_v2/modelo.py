"""go2_modelo: los angulos de las 12 patas del perro -> /joint_states, para que el modelo 3D del Go2
(urdf/go2.urdf, robot_state_publisher) se mueva en RViz igual que el perro de verdad.

  /lf/lowstate (unitree_go/LowState, angulos de los motores) -> /joint_states (~20 Hz)
  Sin datos del perro (simulador, o todavia no llegan) -> el perro DE PIE, quieto.

El modelo va pegado al marco 'vehicle' de CMU (el centro del cuerpo) por un TF fijo del launch.
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

# orden de los motores del Go2 en LowState.motor_state (unitree_sdk2) = nombres del urdf
JUNTAS = ["%s_%s_joint" % (pata, j) for pata in ("FR", "FL", "RR", "RL") for j in ("hip", "thigh", "calf")]
DE_PIE = [0.0, 0.8, -1.5] * 4


class Modelo(Node):
    def __init__(self):
        super().__init__("go2_modelo")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.t_real = 0.0
        self.t_pub = 0.0
        try:
            from unitree_go.msg import LowState
            self.create_subscription(LowState, "/lf/lowstate", self.cb_low, qos_profile_sensor_data)
        except ImportError:
            self.get_logger().warn("sin unitree_go: el modelo queda de pie")
        self.create_timer(0.5, self.de_pie)

    def publicar(self, q):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(JUNTAS)
        js.position = [float(v) for v in q]
        self.pub.publish(js)

    def cb_low(self, m):
        ahora = time.monotonic()
        if ahora - self.t_pub < 0.05:
            return                                   # 20 Hz alcanza para verlo
        q = [m.motor_state[i].q for i in range(12)]
        if not any(abs(v) > 1e-3 for v in q):
            return                                   # todo cero: el perro todavia no mando angulos
        self.t_real = self.t_pub = ahora
        self.publicar(q)

    def de_pie(self):
        if time.monotonic() - self.t_real > 1.0:
            self.publicar(DE_PIE)


def main():
    rclpy.init()
    n = Modelo()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Prueba de rampa de velocidad contra el motor real.

Sube el target de 0 a target_max (rampa lineal), lo mantiene un rato, y
baja de vuelta a 0 — mientras mide el error de tracking (velocidad medida
vs. target comandado en cada instante).

Uso:
    ros2 run swarmbot_bringup test_wheel_ramp.py --ros-args \
        -p wheel:=wheel_right -p target_max:=3.0

Nota: un PI real SIEMPRE tiene algo de error de seguimiento durante la
parte de pendiente constante de la rampa (error de rampa, es teoria de
control basica para un sistema tipo 1) — no es un bug, es la firma
esperada. Lo que si importa es que el error no crezca sin control ni deje
overshoot grande al llegar a los tramos planos.
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class RampTest(Node):

    def __init__(self):
        super().__init__('wheel_ramp_test')
        self.declare_parameter('robot_name', 'robot_01')
        self.declare_parameter('wheel', 'wheel_right')
        self.declare_parameter('target_max', 3.0)
        self.declare_parameter('ramp_up_s', 3.0)
        self.declare_parameter('hold_s', 1.5)
        self.declare_parameter('ramp_down_s', 3.0)
        self.declare_parameter('command_period_s', 0.05)

        robot_name = self.get_parameter('robot_name').value
        wheel = self.get_parameter('wheel').value
        self.target_max = float(self.get_parameter('target_max').value)
        self.ramp_up_s = float(self.get_parameter('ramp_up_s').value)
        self.hold_s = float(self.get_parameter('hold_s').value)
        self.ramp_down_s = float(self.get_parameter('ramp_down_s').value)
        self.command_period_s = float(self.get_parameter('command_period_s').value)

        prefix = f'{robot_name}/{wheel}'
        self.target_pub = self.create_publisher(Float32, f'{prefix}/target_velocity', 10)
        self.create_subscription(Float32, f'{prefix}/velocity', self._velocity_cb, 10)

        self.samples = []  # (target_comandado, medido) por cada mensaje de velocidad
        self.current_target = 0.0

    def _velocity_cb(self, msg):
        self.samples.append((self.current_target, msg.data))

    def target_at(self, t):
        if t < self.ramp_up_s:
            return self.target_max * (t / self.ramp_up_s)
        t -= self.ramp_up_s
        if t < self.hold_s:
            return self.target_max
        t -= self.hold_s
        if t < self.ramp_down_s:
            return self.target_max * (1.0 - t / self.ramp_down_s)
        return 0.0

    def run(self):
        total_s = self.ramp_up_s + self.hold_s + self.ramp_down_s
        start = time.time()
        msg = Float32()
        while True:
            t = time.time() - start
            if t > total_s:
                break
            self.current_target = self.target_at(t)
            msg.data = self.current_target
            self.target_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=self.command_period_s)

        msg.data = 0.0
        for _ in range(3):
            self.target_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    node = RampTest()
    print(f"Rampa: 0 -> {node.target_max} rad/s en {node.ramp_up_s}s, "
          f"hold {node.hold_s}s, baja en {node.ramp_down_s}s")
    node.run()

    samples = node.samples
    if not samples:
        print("No se recibieron muestras de velocidad (revisar el nombre de rueda/topic).")
    else:
        errors = [m - t for (t, m) in samples]
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        max_abs_err = max(abs(e) for e in errors)
        max_measured = max(m for (_, m) in samples)
        overshoot = max(0.0, max_measured - node.target_max)

        print(f"\nMuestras recibidas: {len(samples)}")
        print(f"RMSE de tracking (toda la rampa, incluye el error de rampa esperado): {rmse:.4f} rad/s")
        print(f"Error absoluto maximo: {max_abs_err:.4f} rad/s")
        print(f"Velocidad medida maxima: {max_measured:.4f} rad/s (target_max={node.target_max})")
        print(f"Overshoot sobre target_max: {overshoot:.4f} rad/s")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

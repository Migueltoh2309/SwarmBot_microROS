#!/usr/bin/env python3
"""Publica solo la orientacion en el eje Z (yaw) del MPU6050 como TF
'<robot_name>/odom' -> '<robot_name>/base_link', sin traslacion.

Robot diferencial con rueda loca: el chasis siempre esta apoyado plano
sobre el piso, asi que roll/pitch no aportan nada util para este robot —
solo se integra angular_velocity.z (gyro). Sin magnetometro no hay forma
de corregir la deriva del yaw con el tiempo; es esperado, no un bug (ver
swarmbot_mpu6050_imu.md).

El MPU6050 tiene un bias de fabrica en el gyro Z (medido en este chip:
~-0.014 rad/s quieto, no es ruido — es un offset constante). Sin
calibrarlo, se integra solo y el robot "gira solo" en RViz aunque el
sensor este perfectamente quieto. Se calibra al arrancar: los primeros
CALIBRATION_SAMPLES mensajes (~1s a 20Hz) se promedian asumiendo que el
sensor esta quieto en ese momento, y ese promedio se resta de ahi en
adelante. Esto NO corrige la deriva de largo plazo (el bias puede variar
un poco con la temperatura), solo el offset inicial — con esto el drift
pasa de ~47 grados/min a un residuo mucho menor.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

CALIBRATION_SAMPLES = 20


class ImuOrientationTF(Node):

    def __init__(self):
        super().__init__('imu_orientation_tf')
        self.declare_parameter('robot_name', 'robot_01')
        robot_name = self.get_parameter('robot_name').value

        self.odom_frame = f'{robot_name}/odom'
        self.base_frame = f'{robot_name}/base_link'

        self.yaw = 0.0
        self.last_time = None
        self.gyro_z_bias = 0.0
        self.calibration_sum = 0.0
        self.calibration_count = 0

        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Imu, f'{robot_name}/imu', self.imu_callback, 10)
        self.get_logger().info(
            f'Calibrando bias del gyro Z ({CALIBRATION_SAMPLES} muestras, '
            f'mantener {robot_name} quieto)...')

    def imu_callback(self, msg: Imu):
        if self.calibration_count < CALIBRATION_SAMPLES:
            self.calibration_sum += msg.angular_velocity.z
            self.calibration_count += 1
            if self.calibration_count == CALIBRATION_SAMPLES:
                self.gyro_z_bias = self.calibration_sum / CALIBRATION_SAMPLES
                self.get_logger().info(
                    f'Calibracion lista: bias gyro Z = {self.gyro_z_bias:.5f} rad/s')
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        self.yaw += (msg.angular_velocity.z - self.gyro_z_bias) * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw * 0.5)
        t.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = ImuOrientationTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Auto-tuning del PI de velocidad de una rueda, contra el motor real.

Prueba una grilla de (kp, ki) publicando en <wheel>/kp, <wheel>/ki y
<wheel>/target_velocity, mide el error contra el setpoint leyendo
<wheel>/velocity, y se queda con la combinacion de menor RMSE en estado
estacionario. Al final dumpea un ranking y dispara la mejor combinacion,
dejando el motor detenido.

Uso (con el agente ya corriendo y la fuente del puente H encendida):
    ros2 run swarmbot_bringup tune_wheel_pi.py --ros-args \
        -p wheel:=wheel_left -p target:=2.0

No requiere reflashear entre iteraciones: kp/ki se cambian en caliente
(ver wheel_left_kp_callback/wheel_left_ki_callback en main.c, que resetean
el integrador del PI cada vez que cambia una ganancia).
"""
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PITuner(Node):

    def __init__(self):
        super().__init__('pi_tuner')
        self.declare_parameter('robot_name', 'robot_01')
        self.declare_parameter('wheel', 'wheel_left')
        self.declare_parameter('target', 2.0)
        self.declare_parameter('settle_s', 1.5)
        self.declare_parameter('measure_s', 2.5)

        robot_name = self.get_parameter('robot_name').value
        wheel = self.get_parameter('wheel').value
        self.target = float(self.get_parameter('target').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.measure_s = float(self.get_parameter('measure_s').value)

        prefix = f'{robot_name}/{wheel}'
        self.kp_pub = self.create_publisher(Float32, f'{prefix}/kp', 10)
        self.ki_pub = self.create_publisher(Float32, f'{prefix}/ki', 10)
        self.target_pub = self.create_publisher(Float32, f'{prefix}/target_velocity', 10)
        self.create_subscription(Float32, f'{prefix}/velocity', self._velocity_cb, 10)

        self.samples = []

    def _velocity_cb(self, msg):
        self.samples.append(msg.data)

    def _publish_once(self, pub, value):
        # Sin QoS transient-local: publicar varias veces por si el
        # suscriptor del ESP32 todavia no hizo match la primera vez.
        msg = Float32()
        msg.data = float(value)
        for _ in range(3):
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_trial(self, kp, ki):
        self._publish_once(self.target_pub, 0.0)
        time.sleep(0.3)
        self._publish_once(self.kp_pub, kp)
        self._publish_once(self.ki_pub, ki)

        self.samples = []
        self._publish_once(self.target_pub, self.target)

        deadline = time.time() + self.settle_s + self.measure_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        # Se descarta la ventana de asentamiento (settle_s); solo se mide
        # el error en las muestras de measure_s mas recientes.
        # A 100Hz (firmware 2026-09-12+), measure_s segundos ~= measure_s*100 muestras.
        n_measure = max(1, int(self.measure_s * 100))
        steady = self.samples[-n_measure:] if len(self.samples) >= n_measure else self.samples

        if not steady:
            return {'kp': kp, 'ki': ki, 'rmse': float('inf'), 'overshoot': float('inf'),
                     'stalled': True, 'n': 0}

        errors = [self.target - v for v in steady]
        rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
        overshoot = max(0.0, max(steady) - self.target)
        # Nunca se movio en toda la ventana (backlash/friccion estatica de la
        # reductora sin vencer): distinto de "se movio pero con error".
        stalled = all(abs(v) < 1e-6 for v in steady)
        return {'kp': kp, 'ki': ki, 'rmse': rmse, 'overshoot': overshoot,
                'stalled': stalled, 'n': len(steady)}

    def run_trial_averaged(self, kp, ki, repeats=2):
        trials = [self.run_trial(kp, ki) for _ in range(repeats)]
        stalled = any(t['stalled'] for t in trials)
        rmse = sum(t['rmse'] for t in trials) / len(trials)
        overshoot = sum(t['overshoot'] for t in trials) / len(trials)
        # Penaliza el overshoot ademas del error de tracking puro — un PI
        # que oscila fuerte alrededor del setpoint puede dar buen RMSE pero
        # es un mal comportamiento para una rueda real.
        score = rmse + 0.5 * overshoot
        return {'kp': kp, 'ki': ki, 'rmse': rmse, 'overshoot': overshoot,
                'stalled': stalled, 'score': score}

    def stop(self):
        self._publish_once(self.target_pub, 0.0)


def main():
    rclpy.init()
    node = PITuner()

    # Recentrado 2026-09-12 tras subir el lazo a 100Hz + filtro de media
    # movil (antes 20Hz sin filtrar): equivalente convertido de las
    # ganancias del PI de referencia en Arduino (Kp=1.2, Ki=5.0 en
    # RPM->PWM 0-255) da kp~0.045, ki~0.19 en rad/s->[-1,1]; se barre
    # alrededor de eso.
    kp_candidates = [0.02, 0.035, 0.05, 0.07, 0.09]
    ki_candidates = [0.08, 0.13, 0.19, 0.26, 0.35]

    results = []
    print(f"Target: {node.target} rad/s | settle={node.settle_s}s measure={node.measure_s}s | 2 repeticiones/combo")
    print(f"{'kp':>6} {'ki':>6} {'rmse':>8} {'overshoot':>10} {'score':>8} {'stalled':>8}")

    try:
        for kp in kp_candidates:
            for ki in ki_candidates:
                r = node.run_trial_averaged(kp, ki, repeats=2)
                results.append(r)
                print(f"{kp:6.3f} {ki:6.3f} {r['rmse']:8.4f} {r['overshoot']:10.4f} "
                      f"{r['score']:8.4f} {str(r['stalled']):>8}")
    finally:
        node.stop()
        time.sleep(0.3)

    # Descarta combos que se atascaron alguna vez (no confiables) antes de
    # ordenar por score.
    usable = [r for r in results if not r['stalled']]
    ranked = sorted(usable, key=lambda r: r['score'])

    print("\n--- Top 5 (menor score = rmse + 0.5*overshoot) ---")
    for r in ranked[:5]:
        print(f"kp={r['kp']:.3f} ki={r['ki']:.3f} rmse={r['rmse']:.4f} "
              f"overshoot={r['overshoot']:.4f} score={r['score']:.4f}")

    if not ranked:
        print("\nNinguna combinacion se movio de forma confiable. Revisar mecanica (fuente, cableado, backlash).")
        node.destroy_node()
        rclpy.shutdown()
        return

    best = ranked[0]
    print(f"\nMEJOR: kp={best['kp']:.3f} ki={best['ki']:.3f} "
          f"rmse={best['rmse']:.4f} overshoot={best['overshoot']:.4f}")
    print("Dejando esas ganancias activas (en caliente) y el motor detenido.")
    node._publish_once(node.kp_pub, best['kp'])
    node._publish_once(node.ki_pub, best['ki'])

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Zapis zdarzeń JOG TCP podczas prób sprzętowych (tylko odczyt ROS, bez komend).

Zapisuje JSONL: zdarzenia /robot/manual/tcp_telemetry (start/fault/stopped/sample) oraz każdą zmianę
komunikatu i awarii /robot/manual/state. Zakończenie: SIGINT.
Użycie: python3 scripts/record_tcp_jog_telemetry.py <plik.jsonl>
"""
import json
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from robot_control_interfaces.msg import ManualState


def main(path):
    rclpy.init()
    node = rclpy.create_node('tcp_jog_trial_recorder')
    out = open(path, 'a', encoding='utf-8', buffering=1)
    previous = [None]

    def telemetry(message):
        out.write(json.dumps({'source': 'tcp_telemetry', 'wall_ns': time.time_ns(), 'data': json.loads(message.data)}, ensure_ascii=False) + '\n')

    def state(message):
        key = (message.fault, message.message, message.recovery_state)
        if key != previous[0]:
            previous[0] = key
            out.write(json.dumps({'source': 'manual_state', 'wall_ns': time.time_ns(), 'fault': message.fault,
                                  'message': message.message, 'recovery_state': message.recovery_state}, ensure_ascii=False) + '\n')

    node.create_subscription(String, '/robot/manual/tcp_telemetry', telemetry,
                             QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(ManualState, '/robot/manual/state', state, 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()


if __name__ == '__main__':
    main(sys.argv[1])

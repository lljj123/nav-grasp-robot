#!/usr/bin/env python3
"""High-visibility terminal view for pick/place mission events."""

from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


RESET = '\033[0m'
BOLD = '\033[1m'
COLORS = {
    'READY': '\033[1;36m',
    'START': '\033[1;96m',
    'WAIT': '\033[1;34m',
    'POSE': '\033[1;95m',
    'NAV': '\033[1;33m',
    'PICK': '\033[1;35m',
    'PLACE': '\033[1;96m',
    'OK': '\033[1;32m',
    'DONE': '\033[1;92m',
    'ERROR': '\033[1;31m',
}
ICONS = {
    'READY': '[READY]', 'START': '[START]', 'WAIT': '[WAIT]',
    'POSE': '[POSE]', 'NAV': '[NAV]', 'PICK': '[PICK]',
    'PLACE': '[PLACE]', 'OK': '[ OK ]', 'DONE': '[DONE]',
    'ERROR': '[ERROR]',
}


class MissionConsole(Node):
    def __init__(self):
        super().__init__('mission_console')
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, '/pick_place_mission/event', self.show, qos)
        print('\033[2J\033[H', end='')
        print(f'{BOLD}\033[1;97mTRACER 2.0  两轮导航抓放任务日志{RESET}')
        print('\033[90m' + '=' * 72 + RESET)
        print('\033[90m等待任务事件；此窗口不会发送控制命令。\n' + RESET, flush=True)

    def show(self, message):
        kind, separator, body = message.data.partition('|')
        if not separator:
            kind, body = 'READY', message.data
        color = COLORS.get(kind, '\033[1;97m')
        icon = ICONS.get(kind, '[INFO]')
        timestamp = datetime.now().strftime('%H:%M:%S')
        print(f'\033[90m{timestamp}{RESET} {color}{icon} {body}{RESET}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MissionConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

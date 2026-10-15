"""Interactive terminal client for deliberate one-sample hand-eye capture."""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class HandEyeCollectClient(Node):
    def __init__(self) -> None:
        super().__init__('handeye_collect')
        self.capture_client = self.create_client(
            Trigger, '/handeye/capture_sample')
        self.solve_client = self.create_client(Trigger, '/handeye/solve')
        self.reset_client = self.create_client(Trigger, '/handeye/reset')

    def wait_until_ready(self) -> bool:
        print('正在等待 /handeye/capture_sample 服务……', flush=True)
        while rclpy.ok():
            if self.capture_client.wait_for_service(timeout_sec=1.0):
                return True
            print('标定节点尚未就绪，请确认 ./start_handeye.sh 正在运行。', flush=True)
        return False

    def call(self, client) -> tuple[bool, str]:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.cancelled():
            return False, '服务请求已取消'
        error = future.exception()
        if error is not None:
            return False, f'服务调用失败：{error}'
        result = future.result()
        return bool(result.success), str(result.message)


def _print_help() -> None:
    print(
        '\n手眼标定交互采样\n'
        '  移动机械臂并等待其完全停止，然后：\n'
        '  [回车]  检查稳定性并保存一组数据\n'
        '  s       使用当前全部样本求解并保存标定结果\n'
        '  r       清空当前全部样本（会再次确认）\n'
        '  h       显示帮助\n'
        '  q       退出交互采样\n',
        flush=True,
    )


def main(args=None) -> None:
    if not sys.stdin.isatty():
        print('handeye_collect 必须在交互终端中运行。', file=sys.stderr)
        raise SystemExit(2)

    rclpy.init(args=args)
    node = HandEyeCollectClient()
    try:
        if not node.wait_until_ready():
            return
        _print_help()
        while rclpy.ok():
            try:
                command = input('机械臂停稳后按回车采样 [s求解/r清空/h帮助/q退出] > ')
            except EOFError:
                break
            command = command.strip().lower()
            if command == 'q':
                break
            if command == 'h':
                _print_help()
                continue
            if command == 's':
                success, message = node.call(node.solve_client)
            elif command == 'r':
                answer = input('确认删除当前全部手眼样本？输入 yes：').strip().lower()
                if answer != 'yes':
                    print('已取消清空。', flush=True)
                    continue
                success, message = node.call(node.reset_client)
            elif command == '':
                success, message = node.call(node.capture_client)
            else:
                print('未知命令；按 h 查看帮助。', flush=True)
                continue

            status = '成功' if success else '未采集'
            print(f'[{status}] {message}\n', flush=True)
    except KeyboardInterrupt:
        print('\n已退出交互采样。', flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

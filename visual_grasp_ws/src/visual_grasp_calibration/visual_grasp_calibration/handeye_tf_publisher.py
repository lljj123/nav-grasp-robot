"""Publish the calibrated flange-to-D435 transform from a YAML result."""

from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class HandEyeTfPublisher(Node):
    def __init__(self) -> None:
        super().__init__('handeye_tf_publisher')
        self.result_path = Path(self.declare_parameter(
            'result_path',
            '~/.config/nav_grasp/calibration/handeye.yaml',
        ).value).expanduser()
        self.broadcaster = StaticTransformBroadcaster(self)
        self.published = False
        self.timer = self.create_timer(2.0, self._try_publish)
        self._try_publish()

    def _try_publish(self) -> None:
        if self.published or not self.result_path.exists():
            return
        try:
            data = yaml.safe_load(self.result_path.read_text(encoding='utf-8')) or {}
            handeye = data.get('handeye', {})
            if not handeye.get('valid', False):
                return
            transform_data = handeye['transform_flange_to_camera_link']
            translation = transform_data['translation_m']
            quaternion = transform_data['rotation_xyzw']
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = handeye['parent_frame']
            message.child_frame_id = handeye['child_frame']
            message.transform.translation.x = float(translation[0])
            message.transform.translation.y = float(translation[1])
            message.transform.translation.z = float(translation[2])
            message.transform.rotation.x = float(quaternion[0])
            message.transform.rotation.y = float(quaternion[1])
            message.transform.rotation.z = float(quaternion[2])
            message.transform.rotation.w = float(quaternion[3])
            self.broadcaster.sendTransform(message)
            self.published = True
            self.get_logger().info(
                f'Published calibrated static TF {message.header.frame_id} -> '
                f'{message.child_frame_id} from {self.result_path}')
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
            self.get_logger().error(f'Invalid hand-eye result {self.result_path}: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

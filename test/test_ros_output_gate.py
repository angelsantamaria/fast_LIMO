"""A calibrated IMU without LiDAR must never produce usable localization."""
import os
from pathlib import Path
import signal
import subprocess
import time

os.environ['ROS_DOMAIN_ID'] = '198'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('freshness_gate,validation', [(False, False), (True, False), (False, True)])
def test_imu_only_is_withheld(freshness_gate, validation):
    rclpy.init()
    node = Node('output_gate_test')
    imu_pub = node.create_publisher(Imu, '/ouster/imu', 10)
    poses, transforms, health = [], [], []
    subscriptions = [
        node.create_subscription(Odometry, '/fast_limo/state', poses.append, 10),
        node.create_subscription(Odometry, '/fast_limo/body', poses.append, 10),
        node.create_subscription(TFMessage, '/tf', transforms.append, 10),
        node.create_subscription(Bool, '/fast_limo/localization_healthy', health.append, 10),
    ]
    executable = Path(get_package_prefix('fast_limo')) / 'lib/fast_limo/fast_limo_multi_exec'
    config = Path(__file__).parents[1] / 'config/params.yaml'
    process = subprocess.Popen(
        [str(executable), '--ros-args', '--params-file', str(config),
         '-p', 'calibration.time:=0.15',
         '-p', f'calibration.freshness_gate:={str(freshness_gate).lower()}',
         '-p', f'calibration.validate_stationary:={str(validation).lower()}',
         '-p', 'calibration.max_startup_retries:=1', '-p', 'verbose:=false', '-p', 'debug:=false'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + (8.0 if validation else 2.5)
        while time.monotonic() < deadline:
            msg = Imu()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.orientation.w = 1.0
            msg.linear_acceleration.z = 9.81
            imu_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.005)
        assert process.poll() == (1 if validation else None)
        assert health and all(not msg.data for msg in health)
        assert not poses and not transforms
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()
    assert ('Estimated initial attitude' in output) == (not freshness_gate), output
    if freshness_gate:
        assert 'Waiting for fresh sensors' in output, output

    if validation:
        assert output.count('Estimated initial attitude') == 2, output
        assert 'Retry limit reached' in output, output

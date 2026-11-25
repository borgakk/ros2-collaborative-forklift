"""
Drive test: command a twist for some seconds, compare Gazebo ground truth with wheel odometry.

Start the robot first, for example:
  ros2 launch forklift_bringup sandbox.launch.py drive_type:=mecanum gui:=false

Usage:
  python3 scripts/drive_test.py <odom_topic> [namespace] [case-filter] [speed-scale]
  python3 scripts/drive_test.py /forklift_base_controller/odometry          # Mecanum robot
  python3 scripts/drive_test.py /forklift_base_controller/odom '' forward   # differential, one case
  python3 scripts/drive_test.py /forklift_base_controller/odometry '' forward 3   # 3x faster, same distance

The commands go to /cmd_vel/teleop (twist_mux input). The displacements are in the robot
frame at the start of each case. The robot must have free space around it: the test moves it
about 2 m forward, 1 m back, 1.5 m left and right, and so on. The differential robot cannot
move sideways: the "strafe" cases show 0 for it, which is correct.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class T(Node):
    def __init__(self, odom_topic, ns):
        super().__init__('motion_test')
        self.gt = None
        self.od = None
        self.create_subscription(Odometry, f'{ns}/ground_truth/odom', self.on_gt, 10)
        self.create_subscription(Odometry, odom_topic, self.on_od, 10)
        self.pub = self.create_publisher(TwistStamped, f'{ns}/cmd_vel/teleop', 10)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_od(self, m):
        p = m.pose.pose
        self.od = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.od_twist = (m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z)

    def spin_for(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, vx, vy, wz, sec):
        self.spin_for(0.3)
        g0, o0 = self.gt, self.od
        end = time.time() + sec
        while time.time() < end:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = vx, vy, wz
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
        # stop and settle
        t_end = time.time() + 1.5
        while time.time() < t_end:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.05)
        return g0, self.gt, o0, self.od

    @staticmethod
    def local(p0, p1):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        c, s = math.cos(-p0[2]), math.sin(-p0[2])
        dyaw = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
        return dx * c - dy * s, dx * s + dy * c, dyaw


def main():
    rclpy.init()
    odom_topic = sys.argv[1]
    ns = sys.argv[2] if len(sys.argv) > 2 else ''
    n = T(odom_topic, ns)
    t0 = time.time()
    while (n.gt is None or n.od is None) and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.gt is None or n.od is None:
        print('NO DATA', 'gt' if n.gt is None else '', 'odom' if n.od is None else '')
        return
    cases = [('forward  vx=+0.4', 0.4, 0.0, 0.0, 5.0),
             ('backward vx=-0.3', -0.3, 0.0, 0.0, 4.0),
             ('strafe L vy=+0.3', 0.0, 0.3, 0.0, 5.0),
             ('strafe R vy=-0.3', 0.0, -0.3, 0.0, 5.0),
             ('rotate   wz=+0.4', 0.0, 0.0, 0.4, 5.0),
             ('diagonal vx=.3 vy=.3', 0.3, 0.3, 0.0, 4.0)]
    only = sys.argv[3] if len(sys.argv) > 3 else ''
    scale = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
    for name, vx, vy, wz, sec in cases:
        if only and only not in name:
            continue
        # A speed scale > 1 drives faster for a shorter time: the distances stay the same.
        vx, vy, wz, sec = vx * scale, vy * scale, wz * scale, sec / scale
        g0, g1, o0, o1 = n.drive(vx, vy, wz, sec)
        gx, gy, gw = n.local(g0, g1)
        ox, oy, ow = n.local(o0, o1)
        print(f'{name:22s} expected(~) dx={vx*sec:+.2f} dy={vy*sec:+.2f} dyaw={wz*sec:+.2f} | '
              f'GROUND TRUTH dx={gx:+.2f} dy={gy:+.2f} dyaw={gw:+.2f} | '
              f'ODOM dx={ox:+.2f} dy={oy:+.2f} dyaw={ow:+.2f}', flush=True)
    rclpy.shutdown()


main()

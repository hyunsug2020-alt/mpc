#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import threading
from collections import deque

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Accel, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class MPCDashboard(Node):
    def __init__(self, cav_id):
        super().__init__(f'mpc_dashboard_cav{cav_id:02d}')

        self.cav_id = cav_id
        id_str = f"{cav_id:02d}"

        # Data buffers (last 500 points)
        self.actual_x = deque(maxlen=500)
        self.actual_y = deque(maxlen=500)
        self.ref_x = deque(maxlen=500)
        self.ref_y = deque(maxlen=500)
        self.pred_x = deque(maxlen=500)
        self.pred_y = deque(maxlen=500)

        self.time = deque(maxlen=500)
        self.velocity = deque(maxlen=500)
        self.angular_vel = deque(maxlen=500)
        self.lateral_error = deque(maxlen=500)

        self.start_time = self.get_clock().now()
        self.current_pose = None

        # Subscribers
        self.create_subscription(
            PoseStamped,
            f'/CAV_{id_str}',
            self.pose_callback,
            10
        )

        self.create_subscription(
            Path,
            f'/local_path_cav{id_str}',
            self.local_path_callback,
            10
        )

        self.create_subscription(
            Path,
            '/mpc_predicted_path',
            self.pred_path_callback,
            10
        )

        self.create_subscription(
            Accel,
            f'/CAV_{id_str}_accel_raw',
            self.accel_callback,
            10
        )

        self.get_logger().info(f'Dashboard started for CAV {cav_id:02d}')

    def pose_callback(self, msg):
        self.current_pose = msg.pose
        self.actual_x.append(msg.pose.position.x)
        self.actual_y.append(msg.pose.position.y)

        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.time.append(t)

    def local_path_callback(self, msg):
        if msg.poses:
            self.ref_x.append(msg.poses[0].pose.position.x)
            self.ref_y.append(msg.poses[0].pose.position.y)

            # Calculate lateral error
            if self.current_pose and len(msg.poses) > 10:
                ref_point = msg.poses[10].pose.position
                dx = ref_point.x - self.current_pose.position.x
                dy = ref_point.y - self.current_pose.position.y
                error = np.sqrt(dx * dx + dy * dy)
                self.lateral_error.append(error)

    def pred_path_callback(self, msg):
        if msg.poses:
            self.pred_x.clear()
            self.pred_y.clear()
            for pose in msg.poses:
                self.pred_x.append(pose.pose.position.x)
                self.pred_y.append(pose.pose.position.y)

    def accel_callback(self, msg):
        self.velocity.append(msg.linear.x)
        self.angular_vel.append(msg.angular.z)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--cav_id', type=int, default=1, help='CAV ID to monitor')
    args = parser.parse_args()

    rclpy.init()
    dashboard = MPCDashboard(args.cav_id)

    # Setup plot
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'MPC Dashboard - CAV {args.cav_id:02d}', fontsize=16)

    def animate(_):
        # Plot 1: XY Trajectory
        ax1.clear()
        if dashboard.actual_x:
            ax1.plot(list(dashboard.actual_x), list(dashboard.actual_y),
                     'b-', label='Actual', linewidth=2)
        if dashboard.ref_x:
            ax1.plot(list(dashboard.ref_x), list(dashboard.ref_y),
                     'g--', label='Reference', linewidth=1)
        if dashboard.pred_x:
            ax1.plot(list(dashboard.pred_x), list(dashboard.pred_y),
                     'r:', label='Predicted', linewidth=2)

        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory (XY)')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')

        # Plot 2: Velocity
        ax2.clear()
        if dashboard.time and dashboard.velocity:
            ax2.plot(list(dashboard.time), list(dashboard.velocity), 'b-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Velocity (m/s)')
        ax2.set_title('Linear Velocity')
        ax2.grid(True)
        ax2.set_ylim([0, 2.0])

        # Plot 3: Angular Velocity
        ax3.clear()
        if dashboard.time and dashboard.angular_vel:
            ax3.plot(list(dashboard.time), list(dashboard.angular_vel), 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Angular Velocity (rad/s)')
        ax3.set_title('Angular Velocity')
        ax3.grid(True)
        ax3.set_ylim([-3, 3])

        # Plot 4: Lateral Error
        ax4.clear()
        if dashboard.time and dashboard.lateral_error:
            ax4.plot(list(dashboard.time), list(dashboard.lateral_error), 'g-', linewidth=2)

            # Warning threshold
            ax4.axhline(y=0.3, color='orange', linestyle='--', label='Warning (0.3m)')
            ax4.axhline(y=0.5, color='red', linestyle='--', label='Critical (0.5m)')

            # Highlight violations
            times = list(dashboard.time)
            errors = list(dashboard.lateral_error)
            for t, e in zip(times, errors):
                if e > 0.5:
                    ax4.plot(t, e, 'r*', markersize=10)

        ax4.set_xlabel('Time (s)')
        ax4.set_ylabel('Lateral Error (m)')
        ax4.set_title('Tracking Error')
        ax4.legend()
        ax4.grid(True)
        ax4.set_ylim([0, 1.0])

        # Stats in title
        if dashboard.lateral_error:
            mean_err = np.mean(list(dashboard.lateral_error))
            max_err = np.max(list(dashboard.lateral_error))
            fig.suptitle(
                f'MPC Dashboard - CAV {args.cav_id:02d} | '
                f'Mean Error: {mean_err:.3f}m | Max Error: {max_err:.3f}m',
                fontsize=14
            )

    animation.FuncAnimation(fig, animate, interval=100)  # 10Hz update

    plt.tight_layout()

    # Run ROS2 spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(dashboard,), daemon=True)
    spin_thread.start()

    plt.show()

    dashboard.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from pynput import keyboard
import math

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

STEP = 0.05  # radians (~3 deg)

class Teleop(Node):
    def __init__(self):
        super().__init__('xyz_step')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10)

        self.joints = None
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)

        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

        print("\nControls:")
        print("1/2 → joint1")
        print("3/4 → joint2")
        print("5/6 → joint3")
        print("7/8 → joint4")
        print("9/0 → joint5")
        print("-/= → joint6\n")

    def js_cb(self, msg):
        name_to_pos = dict(zip(msg.name, msg.position))
        self.joints = [name_to_pos[j] for j in JOINT_NAMES]

    def send(self):
        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = self.joints
        pt.time_from_start.sec = 1
        traj.points = [pt]

        self.pub.publish(traj)

    def on_press(self, key):
        if self.joints is None:
            return

        try:
            k = key.char
        except:
            return

        if k == '1':
            self.joints[0] += STEP
        elif k == '2':
            self.joints[0] -= STEP
        elif k == '3':
            self.joints[1] += STEP
        elif k == '4':
            self.joints[1] -= STEP
        elif k == '5':
            self.joints[2] += STEP
        elif k == '6':
            self.joints[2] -= STEP
        elif k == '7':
            self.joints[3] += STEP
        elif k == '8':
            self.joints[3] -= STEP
        elif k == '9':
            self.joints[4] += STEP
        elif k == '0':
            self.joints[4] -= STEP
        elif k == '-':
            self.joints[5] += STEP
        elif k == '=':
            self.joints[5] -= STEP
        else:
            return

        self.send()

def main():
    rclpy.init()
    node = Teleop()
    rclpy.spin(node)

if __name__ == "__main__":
    main()

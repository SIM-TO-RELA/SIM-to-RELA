import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pynput import keyboard
import threading
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class UR10_keyboard_teleop(Node):
    
    def __init__(self):
        super().__init__("UR10_keyboard_teleop")

        self.pub = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10
        )

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        self.current_q = [0.0]*6
        self.velocities = [0.0]*6   # rad/s for each joint

        # Velocity scale (rad/s per key)
        self.vel_scale = 0.0001
        # Timer period (Hz)
        self.dt = 0.2  # 50 Hz
        
        # Keyboard listener
        self.pressed_keys = set()
        self.lock = threading.Lock()

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.daemon = True
        listener.start()

       # Timer for velocity integration
        self.create_timer(self.dt, self.velocity_step)
   
    def send_joint_trajectory(self, joint_idx, dq):
        q = self.current_q[:]
        q[joint_idx] += dq

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start.sec = 1
        traj.points.append(pt)

        self.pub.publish(traj)
        self.current_q = q

    def velocity_step(self):
        """Integrate velocities into positions and publish."""
        with self.lock:
            dq = [v * self.dt for v in self.velocities]

            print(dq)

        q = [q + dq for q, dq in zip(self.current_q, dq)]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start.sec = int(self.dt)
        pt.time_from_start.nanosec = int((self.dt - int(self.dt)) * 1e9)
        traj.points.append(pt)

        self.pub.publish(traj)
        self.current_q = q


    def on_press(self, key: keyboard.Key):
        with self.lock:
            self.pressed_keys.add(key)

        if key == keyboard.Key.up:
            self.velocities[1] =  self.vel_scale
            print("up pressed: shoulder_lift_joint +")
        elif key == keyboard.Key.down:
            self.velocities[1] = -self.vel_scale
            print("down pressed: shoulder_lift_joint -")


    def on_release(self, key: keyboard.Key):
        with self.lock:
            self.pressed_keys.discard(key)
        if key == keyboard.Key.up and self.velocities[1] > 0:
            self.velocities[1] = 0.0
        elif key == keyboard.Key.down and self.velocities[1] < 0:
            self.velocities[1] = 0.0


def main(args=None):
    rclpy.init(args=args)
    node = UR10_keyboard_teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
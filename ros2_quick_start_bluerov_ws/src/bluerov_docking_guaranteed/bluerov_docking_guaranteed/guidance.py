import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64
import numpy as np
import time
from scipy.spatial.transform import Rotation as R


class ROVguidance(Node):
    def __init__(self):
        super().__init__('rov_guidance')
        self.ns = self.get_namespace()
        # Publisher du ROV actif
        self.pub_guidance  = self.create_publisher(Twist, self.ns+'/guidance', 10)
        self.pub_guidance_WorldFrame = self.create_publisher(Twist, self.ns+'/guidance_world_frame', 10)
        self.pub_trueHeading = self.create_publisher(Float64, self.ns+'/true_heading', 10)

        self.timer = self.create_timer(0.1, self.run)

        # Subscriber
        self.sub_pursuer_pose         = self.create_subscription(Pose,    self.ns+'/pursuer_pose', self.pursuer_callback, 10)
        self.depth_sub                = self.create_subscription(Float64, self.ns+'/sensors/global_position/rel_alt',     self.callback_depth,   10)
        self.compass_sub              = self.create_subscription(Float64, self.ns+'/sensors/global_position/compass_hdg', self.callback_heading, 10)
        self.pursuer_desired_pose_sub = self.create_subscription(Pose,    self.ns+'/pursuer_desired_pose', self.callback_pursuer_desired_pose, 10)

        self.sub_pursuer_pose
        self.depth_sub
        self.compass_sub
        self.pursuer_desired_pose_sub

        self.estimated_pursuer_pose = Pose()
        self.desired_pursuer_pose   = Pose()

        self.desired_pursuer_pose.position.x = 8.0
        self.desired_pursuer_pose.position.y = 10.0
        self.desired_pursuer_pose.position.z = -1.5

        self.errWorld = Twist()
        self.err = Twist()
        self.depth = 0.0
        self.heading = Float64()

    def callback_pursuer_desired_pose(self,msg):
        self.desired_pursuer_pose = msg

    def callback_depth(self, msg):
        self.depth = msg.data

    def pursuer_callback(self, msg):
        self.estimated_pursuer_pose = msg

    def callback_heading(self, msg):
        self.heading.data = 2*np.pi-msg.data*np.pi/180
        self.pub_trueHeading.publish(self.heading)


    def run(self):
        # Quaternion désiré
        qd = [
            self.desired_pursuer_pose.orientation.x,
            self.desired_pursuer_pose.orientation.y,
            self.desired_pursuer_pose.orientation.z,
            self.desired_pursuer_pose.orientation.w
        ]

        # Quaternion estimé
        qe = [
            self.estimated_pursuer_pose.orientation.x,
            self.estimated_pursuer_pose.orientation.y,
            self.estimated_pursuer_pose.orientation.z,
            self.estimated_pursuer_pose.orientation.w
        ]

        Rd = R.from_quat(qd)
        Re = R.from_quat(qe)

        R_err = Rd * Re.inv()

        err_roll, err_pitch, err_yaw = R_err.as_euler('xyz')
        #self.err.angular.x = err_pitch
        #self.err.angular.y = err_roll
        #self.err.angular.z = err_yaw

        roll,pitch,yaw = Re.as_euler('xyz')

        yaw_desire = 2.0*np.arctan2(self.desired_pursuer_pose.orientation.z,self.desired_pursuer_pose.orientation.w)

        #self.get_logger().info(f"angle_vise: {angle_vise*180/np.pi}")
        #self.get_logger().info(f"yaw_desire: {yaw_desire}")

        # Erreur dans le repère monde
        W_ex = self.desired_pursuer_pose.position.x - self.estimated_pursuer_pose.position.x
        W_ey = self.desired_pursuer_pose.position.y - self.estimated_pursuer_pose.position.y
        W_ez = self.desired_pursuer_pose.position.z - self.depth
        W_eyaw = yaw_desire - self.heading.data

        

        self.errWorld.linear.x  = W_ex
        self.errWorld.linear.y  = W_ey
        self.errWorld.linear.z  = W_ez
        self.errWorld.angular.z = W_eyaw
        self.pub_guidance_WorldFrame.publish(self.errWorld)

        # Erreur dans le repère robot
        self.err.linear.x = np.cos(self.heading.data) * W_ex - np.sin(self.heading.data) * (-W_ey)
        self.err.linear.y = np.sin(self.heading.data) * W_ex + np.cos(self.heading.data) * (-W_ey)
        self.err.linear.z = W_ez
        self.pub_guidance.publish(self.err)


def main(args=None):
    rclpy.init(args=args)
    node = ROVguidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
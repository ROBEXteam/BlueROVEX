import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
import numpy as np
import time
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav
import pymap3d as pm
from std_msgs.msg import Float64


class ROVnav(Node):
    def __init__(self):
        super().__init__('rov_navigation')
        self.ns = self.get_namespace()
        # Publisher du ROV actif
        self.pub_target_pose  = self.create_publisher(Pose, self.ns+'/target_pose', 10)
        self.pub_target_pose_future  = self.create_publisher(Pose, self.ns+'/target_pose_future', 10)

        self.pub_pursuer_pose = self.create_publisher(Pose, self.ns+'/pursuer_pose', 10)
        self.timer = self.create_timer(0.1, self.run)

        # Subscriber au joystick
        self.sub_quat         = self.create_subscription(SbgEkfEuler, self.ns+'/sbg/ekf_euler',  self.quat_callback,           10)
        self.sub_position     = self.create_subscription(SbgEkfNav,   self.ns+'/sbg/ekf_nav',    self.nav_callback,            10)
        self.timer_sub        = self.create_subscription(Float64,     '/mission_timer',   self.callback_timer,         10)
        self.heading_target_sub = self.create_subscription(Float64,   self.ns+'/target_heading', self.callback_target_heading, 10)

        self.timer_sub
        self.target_pose = Pose()
        self.target_pose_future = Pose()
        self.pursuer_pose = Pose()
        self.pursuer_poseLatLon = Pose()

        self.isInitialLatLonGet = False

        #self.lat0,self.lon0, self.alt0 = 48.419229, -4.46918, 94.0
        self.lat0,self.lon0, self.alt0 = 48.41923, -4.46916, 94.0

        self.xt0, self.yt0, self.phit0 = 1.0, 6.0, 0.0
        self.xt, self.yt, self.phit = 0.0, 0.0, 0.0*np.pi/180

        self.vt = 0.3 #m/s
        self.mission_time = 0.0
        self.time_offset = 30.0  # offset in seconds

    def quat_callback(self, msg):
        #self.get_logger().info(f"Received euler angles: roll={msg.angle.x:.2f}, pitch={msg.angle.y:.2f}, yaw={msg.angle.z:.2f}")
        self.pursuer_pose.orientation.x = msg.angle.x
        self.pursuer_pose.orientation.y = msg.angle.y
        self.pursuer_pose.orientation.z = msg.angle.z
        self.pursuer_pose.orientation.w = 0.0 # on n'utilise pas

    def nav_callback(self, msg):
        self.pursuer_poseLatLon.position.x = msg.latitude
        self.pursuer_poseLatLon.position.y = msg.longitude
        self.pursuer_poseLatLon.position.z = msg.altitude

    def callback_timer(self, msg):
        #self.get_logger().info(f"self.mission_time")
        #self.get_logger().info(f"self.mission_time {msg.data}")
        self.mission_time = msg.data 

    def callback_target_heading(self, msg):
        self.phit0 = msg.data

    def run(self):
        
        # get target position
        
        self.xt                        = self.xt0 + self.vt * self.mission_time * np.cos(self.phit0)
        self.yt                        = self.yt0 + self.vt * self.mission_time * np.sin(self.phit0)
        self.phit                      = self.phit0
        self.target_pose.position.x    = self.xt
        self.target_pose.position.y    = self.yt   
        self.target_pose.orientation.z = np.sin(self.phit/2)
        self.target_pose.orientation.w = np.cos(self.phit/2)


        self.xt_future                        = self.xt0 + self.vt * (self.mission_time+self.time_offset) * np.cos(self.phit0)
        self.yt_future                        = self.yt0 + self.vt * (self.mission_time+self.time_offset) * np.sin(self.phit0)
        self.target_pose_future.position.x    = self.xt_future
        self.target_pose_future.position.y    = self.yt_future
        self.target_pose_future.orientation.z = np.sin(self.phit/2)
        self.target_pose_future.orientation.w = np.cos(self.phit/2)



        #self.get_logger().info(f"Pursuer latlon position: x={self.pursuer_poseLatLon.position.x:.2f}, y={self.pursuer_poseLatLon.position.y:.2f}, z={self.pursuer_poseLatLon.position.z:.2f}")
        #self.get_logger().info(f"latlon0 position: x={self.lat0:.2f}, y={self.lon0:.2f}, z={self.alt0:.2f}")
        lat,lon,alt = pm.geodetic2enu( 
            self.pursuer_poseLatLon.position.x, 
            self.pursuer_poseLatLon.position.y, 
            self.pursuer_poseLatLon.position.z, 
            self.lat0, 
            self.lon0, 
            self.alt0 )
        
        self.pursuer_pose.position.x, self.pursuer_pose.position.y, self.pursuer_pose.position.z = lon, -lat, alt # attention, dans le repere ENU, x est vers l'est et y vers le nord, alors que dans le repere de navigation, x est vers le nord et y vers l'est
        
        # publish poses
        if self.target_pose != Pose(): self.pub_target_pose.publish(self.target_pose)
        if self.target_pose_future != Pose():self.pub_target_pose_future.publish(self.target_pose_future)
        if self.pursuer_pose != Pose():self.pub_pursuer_pose.publish(self.pursuer_pose)


def main(args=None):
    rclpy.init(args=args)
    node = ROVnav()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
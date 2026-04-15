import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def rotation_matrix_to_quaternion(R):

    q = np.zeros(4)

    trace = np.trace(R)

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q[3] = 0.25 / s
        q[0] = (R[2,1] - R[1,2]) * s
        q[1] = (R[0,2] - R[2,0]) * s
        q[2] = (R[1,0] - R[0,1]) * s

    else:
        if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
            s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
            q[3] = (R[2,1] - R[1,2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0,1] + R[1,0]) / s
            q[2] = (R[0,2] + R[2,0]) / s

        elif R[1,1] > R[2,2]:
            s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
            q[3] = (R[0,2] - R[2,0]) / s
            q[0] = (R[0,1] + R[1,0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1,2] + R[2,1]) / s

        else:
            s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
            q[3] = (R[1,0] - R[0,1]) / s
            q[0] = (R[0,2] + R[2,0]) / s
            q[1] = (R[1,2] + R[2,1]) / s
            q[2] = 0.25 * s

    return q  # [qx, qy, qz, qw]

def tf2matrix(transform):
    t = transform.transform.translation
    r = transform.transform.rotation

    # translation
    tx, ty, tz = t.x, t.y, t.z

    # quaternion
    qx, qy, qz, qw = r.x, r.y, r.z, r.w

    # matrice de rotation à partir du quaternion
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])

    # matrice homogène
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [tx, ty, tz]

    return T

class TFListener(Node):

    def __init__(self):
        super().__init__('tf_listener_bluerov')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.br = TransformBroadcaster(self)

    def timer_callback(self):

        try:
            map2bluerov_transform = self.tf_buffer.lookup_transform(
                'map',        # parent frame
                'bluerov',    # child frame
                rclpy.time.Time()
            )

            bluerov2aruco_transform = self.tf_buffer.lookup_transform(
                'bluerov',    # parent frame
                'ar_marker_15',      # child frame
                rclpy.time.Time()
            )

            

            T_map2bluerov = tf2matrix(map2bluerov_transform)
            T_bluerov2aruco = tf2matrix(bluerov2aruco_transform)
            #self.get_logger().info(str(np.round(T, 2)))
            T_map_bluerov = T_map2bluerov @ np.linalg.inv(T_bluerov2aruco)

            

            t_msg = TransformStamped()
            #t_msg.header.stamp = stamp
            t_msg.header.frame_id = 'map'
            t_msg.child_frame_id = f'estimated_bluerov'
            t_msg.transform.translation.x = T_map_bluerov[0, 3]
            t_msg.transform.translation.y = T_map_bluerov[1, 3]
            t_msg.transform.translation.z = T_map_bluerov[2, 3]
            # extraction de la rotation en quaternion
            R_map_bluerov = T_map_bluerov[0:3, 0:3]
            q = rotation_matrix_to_quaternion(R_map_bluerov)
            t_msg.transform.rotation.x = float(q[0])
            t_msg.transform.rotation.y = float(q[1])
            t_msg.transform.rotation.z = float(q[2])
            t_msg.transform.rotation.w = float(q[3]) 

            self.br.sendTransform(t_msg)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF non disponible")


def main():
    rclpy.init()

    node = TFListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
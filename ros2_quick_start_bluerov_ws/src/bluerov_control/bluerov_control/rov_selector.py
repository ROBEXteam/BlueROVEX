import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class ROVSelector(Node):
    def __init__(self):
        super().__init__('rov_selector')

        # Paramètre pour le nombre total de ROVs
        self.declare_parameter('Nrov', 1)
        self.Nrov = self.get_parameter('Nrov').value

        # ID courant du ROV
        self.current_id = 1

        # Pour détecter le front montant
        self.prev_button_state = 0

        # Publisher du ROV actif
        self.pub = self.create_publisher(Int32, '/current_rov_id', 10)
        self.publish_current_id()  # envoyer 1 au démarrage

        # Subscriber au joystick
        self.sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(f"ROV Selector ready with {self.Nrov} ROV(s). Starting at 0.")

    def publish_current_id(self):
        msg = Int32()
        msg.data = self.current_id
        self.pub.publish(msg)
        self.get_logger().info(f"Current ROV ID: {self.current_id}")

    def joy_callback(self, msg: Joy):
        # bouton 5 (index 4) sur un joystick standard ROS
        button_index = 5

        # front montant : bouton appuyé maintenant, pas appuyé avant
        if msg.buttons[button_index] == 1 and self.prev_button_state == 0:
            self.current_id += 1
            if self.current_id > self.Nrov:
                self.current_id = 1
            self.publish_current_id()
            

        # mise à jour de l'état précédent
        self.prev_button_state = msg.buttons[button_index]


def main(args=None):
    rclpy.init(args=args)
    node = ROVSelector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
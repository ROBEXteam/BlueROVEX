import sys
import rclpy
from rclpy.node import Node
import time
import sys
sys.path.append("/home/tbelier/Documents/GIT/Drivers/seatrac_ros_driver/src/seatrac/seatrac/")
from seatracLib import Seatrac
from messages.msg import UsblMessage


class Seatrac_node(Node):
    def __init__(self):
        super().__init__('Seatrac')

        # TO CHANGE AS PARAMETERS
        timerDelay = 0.1
        usbName = '/dev/ttyUSB0'

        self.timer = self.create_timer(timerDelay, self.callbackUsbl)
        self.publisher = self.create_publisher(UsblMessage, '/usbl', 10)
        
        self.x150 = Seatrac(usbName)
        self.x150.cidStatusCfgSet()
        self.x150.cidSettingsSet()

        self.t0 = time.time()
        self.IDusblToPing = 2

        self.usblMessage = UsblMessage()

    def callbackUsbl(self):
        data = self.x150.update()
        self.x150.cidPingSend(self.IDusblToPing,Seatrac.MSG_REQU)
        if data != None:
            self.usblMessage.range = float(data["rangeDist"])/10.0
            self.usblMessage.azimuth = float(data["usblAzimuth"])/10.0
            self.usblMessage.elevation = float(data["usblElevation"])/10.0

            self.publisher.publish(self.usblMessage)
               
    def Tcolor(self, text, color):
        if color == "b" or color == "blue": self.get_logger().info(f"\033[94m {text} \033[0m")
        if color == "r" or color == "red": self.get_logger().info(f"\033[91m {text} \033[0m")
        if color == "g" or color == "green": self.get_logger().info(f"\033[92m {text} \033[0m")
        if color == "o" or color == "orange": self.get_logger().info(f"\033[93m {text} \033[0m")
        
def main(args=None):
    rclpy.init(args=args)
    seatrac = Seatrac_node()
    rclpy.spin(seatrac)
    rclpy.shutdown()


if __name__ == "__main__":
    main()








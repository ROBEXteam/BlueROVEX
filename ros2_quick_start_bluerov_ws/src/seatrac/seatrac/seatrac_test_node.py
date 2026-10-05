#-*- coding: utf-8 -*-
import time
from threading import Thread
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
from messages.msg import UsblMessage
from geometry_msgs.msg import Point
from seatracLib import *

class SeatracRxThread(Thread):
    """ """
    def __init__(self,x150):
        super().__init__()
        self.x150 = x150
        self.continueFlag = True

    def run(self):
        while self.continueFlag:
            self.x150.update()

    def stop(self):
        self.continueFlag = False

class SeatracNode(Node):
    """ """
    def __init__(self):
        super().__init__('Seatrac')

        self.seatracSettingsOk = False

        self.declare_parameters( namespace = '', parameters = [
            ('timer_period',0.1),
            ('serial_name','/dev/ttyUSB0'),
            ('serial_baudrate',115200),
            ('beacons_to_track',4), # bit n set -> track beacon n
            ('ping_period',1.0)
        ])
        #
        timerPeriod = self.get_parameter('timer_period').get_parameter_value().double_value 
        self.serialName = self.get_parameter('serial_name').get_parameter_value().string_value
        self.serialBaudrate = self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        beaconBits = self.get_parameter('beacons_to_track').get_parameter_value().integer_value
        self.pingPeriod = self.get_parameter('ping_period').get_parameter_value().double_value

        self.beacons = []
        for beacon in range(1,16):
            if beaconBits & (1 << beacon) > 0:
                self.beacons.append(beacon)

        logger = self.get_logger()
        logger.info('timer_period: {0}'.format(timerPeriod))
        logger.info('serial_name: ' + self.serialName)
        logger.info('serial_baudrate: {0}'.format(self.serialBaudrate))
        logger.info('beacons_to_track: ' + str(self.beacons))
        logger.info('ping_period: {0}'.format(self.pingPeriod))

        self.usblAcoDataPub = self.create_publisher(UsblMessage,'usbl',1)
        self.usblPosDataPub = self.create_publisher(Point,'usbl_pos',1)
        #
        self.pingPeriodSub = self.create_subscription(Float32,'ping_period',self.pingPeriodCbk,1)
        
        self.initSeatrac()

        self.now = time.time()
        self.ago = self.now
        self.beaconIndex = 0
        self.timer = self.create_timer(timerPeriod,self.timerCbk)

    def initSeatrac(self):
        self.x150 = Seatrac(self.serialName,self.serialBaudrate,self.get_logger())

        self.x150.setMsgCbk(Seatrac.CID_SETTINGS_GET,self.seatracSettingsGetCbk)
        self.x150.setMsgCbk(Seatrac.CID_SETTINGS_SET,self.seatracSettingsSetCbk)
        self.x150.setMsgCbk(Seatrac.CID_XCVR_FIX,self.seatracAcoMsgCbk)
        self.x150.setMsgCbk(Seatrac.CID_NAV_QUERY_RESP,self.seatracAcoMsgCbk)

        self.x150.cidSettingsGet()

        self.rxThread = SeatracRxThread(self.x150)
        self.rxThread.start()

    def timerCbk(self):
        self.now = time.time()
        if self.now - self.ago >= self.pingPeriod:
            self.ago = self.now
            if self.beacons != []:
                beaconId = self.beacons[self.beaconIndex]
                self.beaconIndex = (self.beaconIndex + 1) % len(self.beacons)
                self.x150.cidPingSend(beaconId,Seatrac.MSG_REQU)
                #self.x150.cidNavQuerySend(beaconId,{'depth': True,'voltage': True}) #,'attitude': True})
        
    def pingPeriodCbk(self,msg):
        self.pingPeriod = msg.data

    def seatracSettingsGetCbk(self,settings):
        settings.setStatusOutputRate(0.0)
        settings.resetXcvrFlags()
        settings.enableUseAhrsForUsbl(False)
        settings.enableFixMsgs(True)
        #settings.enableUsblMsgs(True)
        settings.restetStatusOutputContents()
        settings.setStatusOutputEnvironment()
        self.x150.cidSettingsSet(settings)

    def seatracSettingsSetCbk(self,result):
        self.seatracSettingsOk = False
        if result == Seatrac.CST_OK:
            self.seatracSettingsOk = True

    def seatracAcoMsgCbk(self,msg):
        usblMsg = UsblMessage()
        usblMsg.header.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info('stamp: ' + str(self.get_clock().now().to_msg()))
        usblMsg.beacon_id = float(msg['srcId'])
        usblMsg.range = msg['rangeDist'] / 10.0
        usblMsg.elevation = msg['usblElevation'] / 10.0
        usblMsg.azimuth = msg['usblAzimuth'] / 10.0
        usblMsg.depth = float(msg['positionDepth']) / 10.0

        # Conversion en radians
        elev = np.radians(usblMsg.elevation)
        azim = np.radians(usblMsg.azimuth)

        self.usblAcoDataPub.publish(usblMsg)

        if usblMsg.range < 0.5: # ignore les mesures trop proches
            self.printColor(f"r={usblMsg.range:.2f} : Ignoring close measurement", "o")
            self.printColor(f"Is a cable between beacon and vehicle ?", "o")
            return
        p = Point()
        # Coordonnées cartésiennes
        p.x = usblMsg.range * np.cos(elev) * np.cos(azim)
        p.y = usblMsg.range * np.cos(elev) * np.sin(azim)
        p.z = usblMsg.range * np.sin(elev)
        
        self.usblPosDataPub.publish(p)


    def stop(self):
        self.rxThread.stop()
        self.rxThread.join()

    def printColor(self, text, color):
        if color == "b" or color == "blue": self.get_logger().info(f"\033[94m {text} \033[0m")
        if color == "r" or color == "red": self.get_logger().info(f"\033[91m {text} \033[0m")
        if color == "g" or color == "green": self.get_logger().info(f"\033[92m {text} \033[0m")
        if color == "o" or color == "orange": self.get_logger().info(f"\033[93m {text} \033[0m")


def main(args=None):
    rclpy.init(args=args)
    seatrac = SeatracNode()
    rclpy.spin(seatrac)
    seatrac.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


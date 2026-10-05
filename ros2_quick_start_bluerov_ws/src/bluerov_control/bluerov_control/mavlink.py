import rclpy
from rclpy.node import Node
from pymavlink import mavutil

from sensor_msgs.msg import Imu, MagneticField, BatteryState
from std_msgs.msg import Float64, String
from messages.msg import State, RCIn, OverrideRCIn, VfrHud
from messages.srv import CommandBool
from rclpy.qos import QoSProfile

import time
import math
import numpy as np
import threading

from scipy.spatial.transform import Rotation as R

def clip(val, min_val, max_val):
    if val <= min_val:
        return min_val
    elif val >= max_val:
        return max_val
    return val

def setup_covariance(cov, stdev):
    var = stdev * stdev
    for i in range(3):
        cov[i, i] = var

def Tcolor(node, text, color):
    if color == "b" or color == "blue": node.get_logger().info(f"\033[94m {text} \033[0m")
    if color == "r" or color == "red": node.get_logger().info(f"\033[91m {text} \033[0m")
    if color == "g" or color == "green": node.get_logger().info(f"\033[92m {text} \033[0m")
    if color == "o" or color == "orange": node.get_logger().info(f"\033[93m {text} \033[0m")
    


class Mavlink(Node):

    MILLIG_TO_MS2 = 9.80665 / 1000.0  # comme dans imu.cpp :contentReference[oaicite:0]{index=0}  
    MILLIMS2_TO_MS2 = 1.0e-3  # conversion des milli-m/s² :contentReference[oaicite:1]{index=1}  
    MILLIRADS_TO_RAD = 1.0e-3  # millirad/s → rad/s :contentReference[oaicite:2]{index=2}  
    GAUSS_TO_TESLA = 1e-4  # Gauss → Tesla :contentReference[oaicite:3]{index=3}  


    def __init__(self):
        super().__init__("mavlink")

        self.declare_parameter('ip_gcs', "192.168.2.1")
        self.declare_parameter('mavlink_port', 14550)
        self.declare_parameter('ip_rov', "192.168.2.2")
        self.declare_parameter('rov', 14550)

        self.ip_gcs = f"udp:{self.get_parameter('ip_gcs').get_parameter_value().string_value}:{self.get_parameter('mavlink_port').get_parameter_value().integer_value}"
        #self.ip_rov = f"tcp:{self.get_parameter('ip_rov').get_parameter_value().string_value}:{self.get_parameter('mavlink_port').get_parameter_value().integer_value}"
        #print(self.ip_rov)
        self.declare_parameter('sysid_mygcs', 255)
        self.sysid_mygcs = self.get_parameter('sysid_mygcs').get_parameter_value().integer_value
        self.declare_parameter('sysid_rov', 1)
        self.sysid_rov = self.get_parameter('sysid_rov').get_parameter_value().integer_value
        
        self.componentid_rov   = 1  
        self.componentid_mygcs = 1 # ne pas changer les componentid, c'est toujours le numéro 1 qui parle (ex: le composant 1 du ROV est son ardupilot, le 200 est peut-être l'IMU etc..)
        self.ns = self.get_namespace()

        #initializing mavlink master
        self.master = mavutil.mavlink_connection(  
            self.ip_gcs,     
            source_system=self.sysid_mygcs,
            source_component=self.componentid_mygcs) # Target component 
        
        self.master.wait_heartbeat()
        self.master.target_component = self.componentid_rov
        self.master.target_system = self.sysid_rov

        print("Attente que le ROV soit prêt...")
        #time.sleep(5)  # 2 secondes suffisent souvent

        # Passage en mode MANUAL
        self.master.set_mode_manual() 

        self.get_logger().info("Connexion MAVLink établie")
        print("target_system : ", self.master.target_system, " / target_component : ", self.master.target_component)
        #print("ip_gcs = ", self.ip_gcs)
        print("sysid_mygcs = ", self.sysid_mygcs)
        print("sysid_rov = ", self.sysid_rov)

        #######################################
        # Désarmement initial
        self.master.mav.command_long_send(
            self.sysid_rov,
            self.componentid_rov,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.armed = False
        print("ROV UNARMED !")

        

        # envoie d'une commande pour mettre les moteurs à l arret 
        rc = [1500] * 18   # 8 canaux
        rc[8] = 1000 # la lumière

        self.command_old = rc
        
        self.master.mav.rc_channels_override_send(
            self.sysid_rov,
            self.componentid_rov,
            *rc
        )

        self.rc = rc
        
        # Covariances (3x3) : initialisation comme dans `setup_covariance()` de imu.cpp :contentReference[oaicite:4]{index=4}
        self.cov_ang = np.zeros((3,3))
        self.cov_acc = np.zeros((3,3))
        self.cov_ori = np.zeros((3,3))
        setup_covariance(self.cov_ang, 0.02)   # std ~ 0.02 rad/s
        setup_covariance(self.cov_acc, 0.1)    # std ~ 0.1 m/s²

        self.imu_pub = self.create_publisher(Imu, 'sensors/imu/data', 10)
        self.imu_raw_pub = self.create_publisher(Imu, 'sensors/imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'sensors/imu/mag', 10)

        ############
        # Publishers
        self.pub_state = self.create_publisher(State, self.ns+"/sensors/state", 10)
        self.pub_battery = self.create_publisher(BatteryState, self.ns+"/sensors/battery", 10)
        self.pub_rel_alt = self.create_publisher(Float64, self.ns+"/sensors/global_position/rel_alt", 10)
        self.pub_heading = self.create_publisher(Float64, self.ns+"/sensors/global_position/compass_hdg", 10)
        self.pub_rc_in = self.create_publisher(RCIn, self.ns+"/sensors/rc/in", 10)
        self.pub_sys_status = self.create_publisher(String, self.ns+"/sensors/sys_status", 10)
        self.pub_ext_state = self.create_publisher(String, self.ns+"/sensors/extended_state", 10)
        self.pub_statustext = self.create_publisher(String, self.ns+"/sensors/statustext/recv", 10)
        self.pub_home_position = self.create_publisher(String, self.ns+"/sensors/home_position/home", 10)  # simplifié
        self.pub_diff_pressure = self.create_publisher(Float64, self.ns+"/sensors/imu/diff_pressure", 10)
        self.pub_temperature_baro = self.create_publisher(Float64, self.ns+"/sensors/imu/temperature_baro", 10)
        self.pub_vfr_hud = self.create_publisher(VfrHud, self.ns+"/sensors/vfr_hud", 10)


        ######## service ###############
        self.srv = self.create_service(CommandBool, 'sensors/cmd/arming', self.manageArming)


        ###### publisher ############
        self.queue_listener = 10
        self.command_pub = self.create_publisher(OverrideRCIn,'sensors/rc/override', self.queue_listener)
        msg = OverrideRCIn()

        msg.channels = [int(1500)] * 18
        self.command_pub.publish(msg)  # on envoie 1 message, juste pour creer le topic. ce ne sera pas ce code qui les enverra, mais plutôt les ecoutera

        ########################

        #### listener
        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.queue_listener = 10
        self.subscription = self.create_subscription( OverrideRCIn, 'sensors/rc/override', self.callback_override, self.qos_profile)
        self.subscription
        timer_period = 0.1/2  # seconds
        self.timer = self.create_timer(timer_period, self.run)

        # Thread MAVLink -> ROS2
        self.thread = threading.Thread(target=self.mavlink_loop, daemon=True)
        self.thread.start()

        self.temps = time.time()

    def callback_override(self, msg):
        #print("Override reçu : ", msg.channels)
        self.rc = msg.channels

    def run(self):  
        self.rc_override(self.rc)
        self.poll()

    def manageArming(self, request, response):
        # Armer si demandé et actuellement désarmé
        if request.value and not self.armed:
            self.set_arm(True)
            self.armed = True
            Tcolor(self, "ROV armed", "g")

        # Désarmer si demandé et actuellement armé
        elif not request.value and self.armed:
            self.set_arm(False)
            self.armed = False
            Tcolor(self, "ROV disarmed", "r")

        response.success = True
        response.result = self.armed

        return response


    def set_arm(self, arm):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0
        )
    
    def rc_override(self, channels):
        #print("Envoi RC override : ", channels)
        rc = channels
        self.master.mav.rc_channels_override_send(
            self.sysid_rov , # self.master.target_system,
            self.componentid_rov, # self.master.target_component,
            *rc
        )
        self.command_old = channels

    def poll(self):
        msg = self.master.recv_match(blocking=False)
        if msg is None:
            return

    def handle_scaled_imu(self, msg):
        # Conversion comme dans imu.cpp handle_scaled_imu :contentReference[oaicite:5]{index=5}
        # Accélération: xacc, yacc, zacc en milli-g → m/s²
        accel = np.array([msg.xacc, msg.yacc, msg.zacc]) * self.MILLIG_TO_MS2
        # Gyro: xgyro, ygyro, zgyro en milli-rad/s → rad/s
        gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro]) * self.MILLIRADS_TO_RAD
        # Champ magnétique: xmag, ymag, zmag en “milli-gauss” (?) converti en Tesla
        mag = np.array([msg.xmag, msg.ymag, msg.zmag]) * self.GAUSS_TO_TESLA

        # Calcul orientation approximative (roll, pitch), yaw = 0 (comme fallback)
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        yaw = 0.0
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        # Header
        t = self.get_clock().now().to_msg()

        # Publisher champ magnétique
        mag_msg = MagneticField()
        mag_msg.header.stamp = t
        mag_msg.header.frame_id = "base_link"
        mag_msg.magnetic_field.x = mag[0]
        mag_msg.magnetic_field.y = mag[1]
        mag_msg.magnetic_field.z = mag[2]
        self.mag_pub.publish(mag_msg)

        # Message IMU
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        # Covariances
        imu_msg.orientation_covariance = [float(self.cov_ori[i,j]) for i in range(3) for j in range(3)]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]
        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]

        self.imu_pub.publish(imu_msg)

        # Message IMU “raw” 
        raw = Imu()
        raw.header = imu_msg.header
        raw.orientation = imu_msg.orientation
        raw.orientation_covariance = imu_msg.orientation_covariance
        raw.angular_velocity = imu_msg.angular_velocity
        raw.angular_velocity_covariance = imu_msg.angular_velocity_covariance
        raw.linear_acceleration = imu_msg.linear_acceleration
        raw.linear_acceleration_covariance = imu_msg.linear_acceleration_covariance
        self.imu_raw_pub.publish(raw)

    def handle_highres_imu(self, msg):
        # HIGHRES_IMU :
        accel = np.array([msg.xacc, msg.yacc, msg.zacc])  # m/s²
        gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro])  # rad/s
        mag = np.array([msg.xmag, msg.ymag, msg.zmag]) * self.GAUSS_TO_TESLA

        # Orientation fallback comme scaled imu
        roll = math.atan2(accel[1], accel[2])
        pitch = math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2))
        yaw = 0.0
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        t = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]
        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]

        imu_msg.orientation_covariance = [float(self.cov_ori[i,j]) for i in range(3) for j in range(3)]
        self.imu_pub.publish(imu_msg)

    def handle_attitude(self, msg):
        # ATTITUDE : contient roll, pitch, yaw (rad/s), transform de NED vers ENU, etc. :contentReference[oaicite:8]{index=8}
        # msg.roll, msg.pitch, msg.yaw : en rad
        # msg.rollspeed, pitchspeed, yawspeed : rad/s
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw

        # Conversion des angles pour passer de NED → ENU, puis vers base_link (frame) selon imu.cpp :contentReference[oaicite:9]{index=9}
        # Ici on effectue un “transform_orientation_ned_enu” implicite : NED (FCU) → ENU
        # Puis un “transform_orientation_aircraft_baselink” = identité si on considère base_link comme frame d’IMU
        quat = R.from_euler('xyz', [roll, pitch, yaw]).as_quat()

        t = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = t
        imu_msg.header.frame_id = "base_link"
        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.orientation_covariance = [1.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0,
                                          0.0, 0.0, 1.0]

        imu_msg.angular_velocity.x = msg.rollspeed
        imu_msg.angular_velocity.y = msg.pitchspeed
        imu_msg.angular_velocity.z = msg.yawspeed
        imu_msg.angular_velocity_covariance = [float(self.cov_ang[i,j]) for i in range(3) for j in range(3)]

        imu_msg.linear_acceleration_covariance = [float(self.cov_acc[i,j]) for i in range(3) for j in range(3)]
        # On ne publie pas les accélérations depuis ATTITUDE
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.imu_pub.publish(imu_msg)

    def mavlink_loop(self):
        while rclpy.ok():
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg is None:
                continue
            
            if msg.get_srcSystem() != self.sysid_rov or msg.get_srcComponent() not in [1, 250]:
                continue


            msg_type = msg.get_type()

            # ================
            # STATE
            # ================
            if msg_type == "HEARTBEAT":
                rosmsg = State()

                rosmsg.header.stamp = self.get_clock().now().to_msg()

                # Armed/désarmé
                rosmsg.armed = (msg.base_mode &
                                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                self.armed = rosmsg.armed 
                # Mode (custom)
                rosmsg.mode = str(msg.custom_mode)

                # system_status = uint8 → OK
                rosmsg.system_status = int(msg.system_status)

                # Champs MAVROS que nous simulons :
                rosmsg.connected = True
                rosmsg.guided = (msg.base_mode &
                                mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) != 0
                rosmsg.manual_input = True

                # Selon ROS2, pas de mode_id → ne pas l’envoyer

                self.pub_state.publish(rosmsg)

            # ================
            # BATTERY STATUS
            # ================
            elif msg_type == "BATTERY_STATUS":
                rosmsg = BatteryState()
                rosmsg.voltage = msg.voltages[0] / 1000.0
                rosmsg.percentage = msg.battery_remaining / 100.0
                self.pub_battery.publish(rosmsg)

            # ================
            # IMU RAW
            # ================

            elif msg_type == 'SCALED_IMU':
                self.handle_scaled_imu(msg)
            elif msg_type == 'HIGHRES_IMU':
                self.handle_highres_imu(msg)
            elif msg_type == 'ATTITUDE':
                self.handle_attitude(msg)



            # ================
            # ALTITUDE
            # ================
            elif msg_type == "GLOBAL_POSITION_INT":
                alt = msg.relative_alt / 1000.0
                self.pub_rel_alt.publish(Float64(data=alt))
                self.pub_heading.publish(Float64(data=msg.hdg / 100.0))

            # ================
            # SYSTEM STATUS
            # ================
            elif msg_type == "SYS_STATUS":

                self.pub_sys_status.publish(String(data=str(msg)))

            # ================
            # EXTENDED STATE
            # ================
            elif msg_type == "EXTENDED_SYS_STATE":
                self.pub_ext_state.publish(String(data=str(msg)))

            elif msg_type == "STATUSTEXT":
                st = String()
                st.data = msg.text
                self.pub_statustext.publish(st)

            elif msg_type == "HOME_POSITION":
                hp = String()
                hp.data = str(msg)  # pour simplifier, tu peux créer un message spécifique si besoin
                self.pub_home_position.publish(hp)

            elif msg_type == "SCALED_PRESSURE":
                diff_pressure = Float64(data=float(msg.press_diff))
                static_pressure = Float64(data=float(msg.press_abs))
                temperature_baro = Float64(data=float(msg.temperature))  # CAST EN FLOAT !

                self.pub_diff_pressure.publish(diff_pressure)
                self.pub_temperature_baro.publish(temperature_baro)

            elif msg_type == "VFR_HUD":
                rosmsg = VfrHud()
                rosmsg.climb = float(msg.climb)              # m/s #verticalSpeed
                self.pub_vfr_hud.publish(rosmsg)


def main():
    rclpy.init()
    node = Mavlink()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

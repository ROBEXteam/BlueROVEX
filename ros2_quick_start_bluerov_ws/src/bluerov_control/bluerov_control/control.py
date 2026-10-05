

# Copyright 2024, Christophe VIEL
# 
# Redistribution and use in source and binary forms, with or without modification, are permitted # provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


# Note : by default, angles provid by sensor are in deg. Variable with a "rad" are expressed in radian

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import time
import datetime
import numpy as np
import subprocess, os, signal, psutil
from scipy.spatial.transform import Rotation 

from messages.msg import OverrideRCIn
from messages.srv import CommandBool

from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64, String, Bool, Float32MultiArray, Int32
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Trigger
from messages.srv import Wset
def sawtooth(x):
    return np.arctan2(np.sin(x), np.cos(x))

class Control(Node):

    def __init__(self):
        super().__init__('Control')

        self.dt = 1/10
        self.ns = self.get_namespace()
        self.declare_parameter('rov_id', 1)
        self.rov_id = self.get_parameter('rov_id').value

        # Robot modes
        self.isArmed = False
        self.program_B = False
        self.name_program_B = "(pas de programme)"

        # Robot parameter
        self.depth = 0.0 # Depth
        self.heading = 0.0 # Heading

        # IMU
        self.phi_rad = 0.0
        self.theta_rad = 0.0
        self.psy_rad = 0.0
        self.angular_velocity_y = 0.0  # vitesse tangage
        self.angular_velocity_x = 0.0  # vitesse rouli
        

        # Buttons and Axes
        if self.rov_id == 1: self.active = True # pour savoir si le rov écoute les commandes, en fonction du ROV sélectionné
        else: self.active = False
        self.letter2index  = {"A": 0, "B": 1, "X": 2, "Y": 3, "LH": 4, "RH": 5, "Back": 6, "Start": 7, "?": 8, "L3": 9, "R3": 10}
        self.buttons       = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # A, B, X, Y, LH , RH , back, start, ?, L3, R3
        self.axes          = [0, 0, 0, 0, 0, 0, 0, 0]
        

        # enregistrement
        self.start_record = False
        self.record = False
        self.record_data = False
        self.choix_record = 0.0 # 1: record all / 2: record datas without cams / 0 : sans choix encore 
        self.nb_choix_record = 2.0 
        self.record_step = 0.0 # etape de selection du mode d'enregistrement
          
        self.isAutonomous, self.hasResponded, self.isArmed = False, False, False


        # ROS
        #### listener
        self.qos_profile = QoSProfile( reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1 )
        self.queue_listener = 10
        self.listener()  # Subscribes to the messages

        # ----------------------- publishers -----------------------#
        self.choice_record_pub  = self.create_publisher(Float32MultiArray, self.ns+"/choice_record",            self.queue_listener) 
        self.enregistrement_pub = self.create_publisher(Bool,              self.ns+'/enregistrement_en_cours',  self.queue_listener)
        self.command_pub        = self.create_publisher(OverrideRCIn,      self.ns+'/sensors/rc/override',      self.queue_listener)
        self.program_B_name_pub = self.create_publisher(String,            self.ns+"/program_B_name",           self.queue_listener) 
        self.is_autonomous_pub  = self.create_publisher(Int32,             self.ns+"/is_autonomous",            self.queue_listener) 
        

        #####################
        self.nb_channels = 18 # rospy.get_param(rospy.get_name() + "/nb_channels", 18) # TODO: chercher pk ca ne marche pas...
        self.commands = [65535] * self.nb_channels
        
        # Mavlink
        self.client_arm = self.create_client(CommandBool, self.ns+'/sensors/cmd/arming')
        self.send_request_arm(False)

        timer_period = self.dt # seconds
        self.timer = self.create_timer(timer_period, self.run)



        # variables a initialiser : 
        self.dt = 1/10  # mettre le temps du systeme
        self.theta_target_rad = 0.0 # pitch desire par defaut, a l'horizontal. On peut choisir une autre valeur a piloter
        self.phi_target_rad = 0.0 # roll desire par defaut, a l'horizontal. On peut choisir une autre valeur a piloter
        self.depth_target = 3.5 # a choisir. Ici en deg vu que le cap (self.heading) est mesure en deg
        self.heading_target_deg = 270 # a choisir. Ici en deg vu que le cap (self.heading) est mesure en deg (N = 0, E = 90, S = 180, W = -90 ou 270, coin NW = -31, 160)
        self.speed = 1650
        self.heading2North = 0#20

        self.x_target = 0.0
        self.y_target = 0.0
        self.estimated_x, self.estimated_y, self.estimated_z = 0.0, 0.0, 0.0 # position estimee
        self.err_guidance = Twist() # erreur de guidage, pour le guidance_node

        # Client de service
        self.client = self.create_client(Wset, self.ns+'/wset_compute')

        # Attendre que le service existe
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service non disponible...')
        
    def listener(self):
        self.compass_sub      = self.create_subscription(Float64, self.ns+'/sensors/global_position/compass_hdg', self.callback_heading, self.qos_profile)
        self.depth_sub        = self.create_subscription(Float64, self.ns+'/sensors/global_position/rel_alt',     self.callback_depth,   self.qos_profile)
        self.subscription     = self.create_subscription(Imu,     self.ns+"/sensors/imu/data",                    self.callback_imu,     self.qos_profile)
        self.joy_sub          = self.create_subscription(Joy,     '/joy',                                         self.callback_joy,     self.queue_listener) #/!\ pas de namespace
        self.rov_selector_sub = self.create_subscription(Int32,   '/current_rov_id',                              self.rov_id_callback,  self.queue_listener) #/!\ pas de namespace
        self.guidance_sub     = self.create_subscription(Twist,   self.ns+'/guidance',                            self.callback_guidance, 10)
        self.pursuer_pose_sub = self.create_subscription(Pose,    self.ns+'/pursuer_pose',                        self.callback_pursuer_pose, 10)
        self.target_pose_sub  = self.create_subscription(Pose,    self.ns+'/target_pose',                         self.callback_target_pose, 10)


        # prevents unused variable warning 
        self.compass_sub 
        self.depth_sub 
        self.joy_sub 
        self.rov_selector_sub
        self.guidance_sub
        self.subscription 
        self.pursuer_pose_sub
        self.target_pose_sub

    def button(self, letter):
        return self.buttons[self.letter2index[letter]]

    def send_commands(self):

        msg = OverrideRCIn()
        for i in range(self.nb_channels):
            if self.commands[i] != 65535: self.commands[i] = np.clip(self.commands[i], 1100, 1900)
            msg.channels[i] = self.commands[i] 
        
        self.command_pub.publish(msg)

        msg = String()
        msg.data = self.name_program_B
        self.program_B_name_pub.publish(msg)
        
        msg1 = [self.record_step,self.choix_record]
        msg  = Float32MultiArray(data = msg1)
        self.choice_record_pub.publish(msg)

        # booléen si on enregistre
        msg = Bool(data = self.start_record)
        self.enregistrement_pub.publish(msg)

    def callback_guidance(self, msg):
        self.err_guidance = msg
    def callback_heading(self, msg):
        self.heading = msg.data+self.heading2North

    def callback_pursuer_pose(self, msg):
        self.pursuer_pose = msg
    
    def callback_target_pose(self, msg):
        self.target_pose = msg

    def callback_heading_reference(self, data):
        self.heading_reference = data.data
        self.heading_global_target = self.heading_reference + self.heading_offset

    def callback_imu(self, msg):
        self.angular_velocity_z = msg.angular_velocity.z  # vitesse de lacet
        self.angular_velocity_y = msg.angular_velocity.y  # (vitesse de tangage ?)
        self.angular_velocity_x = msg.angular_velocity.x  # (vitesse de roulis ?)
        W = msg.orientation.w
        X = msg.orientation.x
        Y = msg.orientation.y
        Z = msg.orientation.z
        orientq=(W, X, Y, Z)
        ### Conversion quaternions in rotation matrix
        self.phi_rad, self.theta_rad, self.psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet 
    

    def rov_id_callback(self, msg):
        self.active = (msg.data == self.rov_id)
        
    def callback_joy(self, msg):
        
        if not self.active: return

        self.old_axes = self.axes
        self.old_buttons = self.buttons

        self.axes = msg.axes
        self.buttons = msg.buttons

        #self.get_logger().info(f"[Button] {self.buttons}")
        #self.get_logger().info(f"[Axis] {self.axes}")

        if self.buttons[7] == 1 and self.old_buttons[7] == 0:  # Start button
            
            self.isArmed = not self.isArmed
            self.get_logger().info(f" {'Armed' if self.isArmed else 'Disarmed'}")
            response = self.send_request_arm(self.isArmed) # armement/desarmement
        
        if self.buttons[0] == 1 and self.old_buttons[0] == 0:  # A button
            self.isAutonomous = not self.isAutonomous
            self.get_logger().info(f"Mode autonome {'activé' if self.isAutonomous else 'désactivé'}")

        if self.buttons[3] == 1 and self.old_buttons[3] == 0:  # Y button
            self.send_request_Wset(self.pursuer_pose, self.target_pose)

    def callback_depth(self, msg):
        self.depth = -msg.data

    def send_request_arm(self, arming):

        while not self.client_arm.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('service not available, waiting again...')
        req = CommandBool.Request()
        req.value = bool(arming)
        self.client_arm.call_async(req)

    def send_request_Wset(self, pose_pursuer, pose_target):

        req = Wset.Request()

        req.pose_pursuer = pose_pursuer
        req.pose_target = pose_target

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        return future.result().pose_result

    def run(self):
        self.commands = [65535] * self.nb_channels
        self.is_autonomous_pub.publish(Int32(data=int(self.isAutonomous)))
        if self.isAutonomous:
            kpi_p, kpi_d = 250, 60
            kro_p, kro_d = 250, 60
            kd_p, kd_d, kd_i = 250.0, 200, 80
            Kp, Kv, Ki = 250, 100, 40 #heading
            kx1, kx2 = 200, 1.0
            ky1, ky2 = 200, 1.0
            kz1, kz2 = 200, 1.0
            
            #### stabilisation en roll
            e = sawtooth(self.theta_rad - self.theta_target_rad)   # note : self.Theta_rad en rad de base,  self.Theta_target_rad en rad
            e_d = self.angular_velocity_y

            u0 = kpi_p*e + kpi_d*e_d
            u0 = np.sign(u0)*min([500, abs(u0)]) # saturation de la commande
            self.commands[0] = int(-u0 + 1500)

            
            ##### stabilisation en pitch
            e = -sawtooth(self.phi_rad - self.phi_target_rad)  
            e_d = -self.angular_velocity_x

            u1 = kro_p*e + kro_d*e_d 
            u1 = np.sign(u1)*min([500, abs(u1)]) 
            self.commands[1] = int(u1 + 1500)
            
            

            """
            #### Commande en profondeur --> marche
            e_old = self.depth - self.depth_target
            e = self.depth - self.depth_target
            e_d = -(e-e_old)/self.dt
            #e_i = e + self.ei_d
            #if abs(e) < 0.5: self.ei_d = e_i
            #else:            self.ei_d = 0.0 # pour eviter d'integrer de trop grande valeur
            
            u2 = (kd_p*e + kd_d*e_d)# + kd_i*self.dt*e_i )
            self.commands[2] = int( u2*np.cos(self.phi_rad) )  + 1500
            """
                            
            ### Commande position
            self.commands[4] = int(1500 + kx1*np.tanh(kx2*self.err_guidance.linear.x))
            self.commands[5] = int(1500 + ky1*np.tanh(ky2*self.err_guidance.linear.y))
            self.commands[2] = int(1500 + kz1*np.tanh(kz2*self.err_guidance.linear.z))


            #self.commands[4] = self.speed


        else:
            
            ##### Lecture des input de la manette
            # Example : move forward/backward
            # INFO :
            # Chaque channel est une valeur PWM :
            # 1000 → minimum
            # 1500 → neutre
            # 2000 → maximum
            # 65535 → ignore ce channel
            roll     = 65535
            pitch    = 65535
            ascend   = int(200 * self.axes[4] + 1500)
            yaw      = int(200 * -self.axes[3] + 1500)
            forward  = int(200 * self.axes[1] + 1500)
            lateral  = int(200 * -self.axes[0] + 1500)
            cam_tilt = 65535
            lights   = 65535
            chan9    = 65535
            chan10   = 65535
            chan11   = 65535
            chan12   = 65535
            chan13   = 65535
            chan14   = 65535
            chan15   = 65535
            chan16   = 65535
            chan17   = 65535
            chan18   = 65535

            self.commands = [roll, pitch, ascend, yaw, forward, lateral, cam_tilt, lights, chan9, chan10, chan11, chan12, chan13, chan14, chan15, chan16, chan17, chan18]




        


        """
        # camera inclination # fleche haut/bas
        if (self.axes[7] != 0):  self.commands[7] = int(100 * self.axes[7] + 1500)
        else:                    self.commands[7] = 1500

        # light control 
        if self.button("?") != 0:  # button Back : control light intensity
            light_modif_value = 100
            light_control(self,light_modif_value)
        ####################################################################
        # selection de l'enregistrement           
        
        if (self.axes[6] != 0)&((self.record_step == 1)|(self.record_step == 2)):
                
            if self.axes[6] > 0.5:
                self.choix_record = 1
                print('record data+cam selected')
                print('Press "Y" again')
                self.record_step = 2 # etape suivante
            elif self.axes[6] < -0.5:
                self.choix_record = 2   
                print('record data without cam selected')
                print('Press "Y" again')
                self.record_step = 2 # etape suivante

        if (self.button("Y") != 0):  # button Y : record
            

            if self.record_step == 0: # si premiere demande

                print('Record : select one choice')
                print(' <- : record data+cam /-> : record data without cam') 
                self.record_step += 1 # demande du choix

            if self.record_step == -1: 
                self.record_step = 0

            # BBYY
            if (self.record_step > 1): # si choix fait OU si enregistrement deja en cours
                

                if (self.choix_record == 1)&(self.record_step == 2): # 0 : par defaut en cas de probleme
                    self.record = not self.record
                    
                    self.record_step = self.record_step+1

                    print('choix 1 fait')
                    time.sleep(0.5)


                elif (self.choix_record == 2)&(self.record_step == 2):
                    self.record_data = not self.record_data

                    self.record_step = self.record_step+1

                    print('choix 2 fait')

                    time.sleep(0.5)


                # si on a demande d'arreter, reset variable    
                elif (self.record_step > 3):

                    self.record_step = -1 # <- pk -1 ? car bug a la ... ou il relance immediatement l'enregistrement. Ca corrige en le faisant passer directement a 0 plutot qu a 1
                    self.choix_record = 0
                    print('Stop recording.')
                    time.sleep(0.5)

                else:
                    self.record_step += 1 # demande du choix

            
            print("self.record_step = ", self.record_step)

        self.function_rosbag()

        """
        ##################################################
        self.send_commands()  # publisher general, pas que commande
        self.commands_old = self.commands



    def function_rosbag(self): #### Pour enregistrement


        if (self.record_step > 2)&(self.start_record == False):
            
            if self.record_data == True:
                add_title = '_without_cam'
            else :
                add_title = ''
            
            # create directory to stock data
            ROOT_DIR = os.path.expanduser('~')
            self.filename0 =  ROOT_DIR + "/quick_start_bluerov_ros2/rosbag/" 

            current_date_and_time = datetime.datetime.now()
            current_date_and_time_string = str(current_date_and_time)[0:10] + '-' +  str(current_date_and_time)[11:13]+'-'+str(current_date_and_time)[14:16]+'-'+ str(current_date_and_time)[17:19]                 #[0:19] : keep only seconds
            path =  self.filename0 + self.ROV_name + add_title +'_' + current_date_and_time_string
            os.mkdir(path)
        
            # start rosbag
            print("Start recording...")
            if self.record_data == True:
                command = 'ros2 bag record -a -x "(.*)/cam_1/compressed|/(.*)/cam_2/compressed"' # enregistre tout sauf les cameras (pour gain de memoire)
                self.record = False # (qu'un seul enregistrement a la fois)
            else:
                command = "ros2 bag record -a"  # enregistre tout
                self.record_data = False  # (qu'un seul enregistrement a la fois)

            
            dir_save_bagfile = path
            self.p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=dir_save_bagfile)
            
            self.path = path # dossier principal d'enregistrement
            self.start_record = True

        elif (self.choix_record == 0)&(self.start_record == True):

            self.start_record = False
            # stop enregistrement rosbag
            self.terminate_process_and_children(self.p)   


########## pour rosbag, terminer l'enregistrement
# https://answers.ros.org/question/10714/start-and-stop-rosbag-within-a-python-script/
    def terminate_process_and_children(self,p):
              
        process = psutil.Process(p.pid)
        for sub_process in process.children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
        p.wait()  # we wait for children to terminate
        p.terminate()

def light_control(self,light_modif_value):


    adresse_ip0 = self.adresse_ip

    pwm_light0 = self.pwm_light+light_modif_value
    if (pwm_light0 > 1900):
        pwm_light0 = 1000.0
    if (pwm_light0 < 1000):
        pwm_light0 = 1900.0

    self.pwm_light = pwm_light0

    
    
    # on ecrit la valeur que l'on veut dans le fichier test.txt   TODO: retirer ce passage pour version publique
    msg_lum = str(pwm_light0) 
    password = 'companion' 
    file3 = '/home/pi/pwm_light.txt'
    cmd = 'sshpass -p '+password+' ssh '+ adresse_ip0 +' "echo '+ msg_lum +' > '+file3+'"'
    os.system(cmd) 

    # for classic bluerov
    if (self.nb_channels > 8): # if the Mavlink version can control the light
        self.commands[8] = self.pwm_light



def main(args=None):
    rclpy.init(args=args)
    node_rov = Control()
    rclpy.spin(node_rov)
    node_rov.destroy_node()
    rclpy.shutdown()

          

if __name__ == '__main__':
    main()
    
    
    

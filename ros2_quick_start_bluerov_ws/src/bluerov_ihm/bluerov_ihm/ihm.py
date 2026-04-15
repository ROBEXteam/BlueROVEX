
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

import cv2
import rclpy
import numpy as np
import tkinter as tk

from rclpy.node import Node
from rclpy.qos import QoSProfile
from messages.msg import State, OverrideRCIn
from sensor_msgs.msg import Imu, Joy, BatteryState, FluidPressure
from std_msgs.msg import Float64, String, Bool, Float32MultiArray

from scipy.spatial.transform import Rotation 


class IHM(Node):
    def __init__(self):
        super().__init__('IHM')

        self.ROV_name = ROV_name = self.get_namespace()
        self.get_logger().info(f"Namespace: {self.ROV_name}")
        
        self.font_scale_2 = 0.75    
        self.size_long    = 800 + 80 
        self.size_lat     = 300 + 10 
        self.nb_channels  = 18 


        # Robot booleans
        self.armed = False
        self.boost = False      
        self.tracking_buoy = False
        self.tracking_with_heading_mode = False
        self.tracking_with_distance_mode = False
        self.tracking_wihtout_depth_control = False

        self.robot_config = "heavy"
        self.robot_mode = "Disarmed"
        
        self.commands = [1500] * self.nb_channels
        if self.nb_channels > 8:     # if mavros version can control light
            self.commands[8] = 1000  # pwm light
        self.light_pwm = 1000
                
        self.heading_offset = 90.0
        
        # telecommande
        self.button = [0] * 8
        self.axes = [0] * 8
        self.piloted = True
        self.joy_active = 1   

        
        self.modeB = False
        self.modeX = False
        self.program_B_name = ""
        
        # enregistrement
        self.record = False
        self.record2 = False
        self.record_step = 0
        self.record_choice_record = 1
        
        # sensors data
        self.pressure = 0
        self.battery_state = 0.
        self.min_battery_state = 15
        self.heading = 0
        self.heading_target = -1
        self.roll_target = -1
        self.pitch_target = -1
        self.depth = 0
        self.depth_target = -1
        self.distance = -1
        self.distance_target = -1
        self.detected = False
        self.detected_x = 0
        self.detected_y = 0
        self.first_detection = 0
        self.distance_ping = 0
        self.confidence_ping = 0
               
        self.Phi = 0
        self.Theta = 0
        self.Psy  = 0  
                
        # IHM params
        self.font = cv2.FONT_HERSHEY_DUPLEX
        self.big_font = cv2.FONT_HERSHEY_TRIPLEX
        self.font_scale = self.font_scale_2 # 0.75
        self.font_thickness = 1
        self.eps = int(30*(self.font_scale_2/0.75)) # 30
        self.big_eps = int(40*(self.font_scale_2/0.75)) #40

        self.red = (60, 76, 231)
        self.green = (113, 204, 46)
        self.purple = (182, 89, 155)
        self.grey = (160, 160, 160)
        self.blue = (219, 152, 52)
        self.white = (255, 255, 255)
        self.yellow = (15, 196, 241)
        self.orange = (34, 126, 230)

        
        # Create IHM window
        cv2.namedWindow("Telemetry ") # +self.ROV_name
        root = tk.Tk()
        width = root.winfo_screenwidth()
        height = root.winfo_screenheight()
        root.destroy()   
        self.ihm_size = (self.size_lat, self.size_long, 3)

        cv2.moveWindow("Telemetry ", 0, height-width)#self.screen_size[1] -self.ihm_size[0])
        #cv2.moveWindow("Telemetry ", self.screen_size[0] - self.ihm_size[1], 0)

        self.ihm_img = np.zeros(self.ihm_size, np.uint8)
        self.x, self.y = 20, 30

        # Joysticks params
        self.R = int(self.ihm_size[1] / 8)
        self.r = int(self.R / 2)


        self.qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.queue_listener = 10
        self.listener()


        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.run)


    # ------------------------ Callbacks ------------------------

    def callback_armed(self, msg):
        self.armed = msg.armed

    def callback_mode(self, msg):
        self.robot_mode = msg.data
        
    def callback_config(self, msg):
        self.robot_config = msg.data

    def callback_compass(self, msg):
        self.heading = msg.data

    def callback_target_heading(self, msg):
        self.heading_target = msg.data

    def callback_target_roll(self, msg):
        self.roll_target = msg.data*180/3.14

    def callback_target_pitch(self, msg):
        self.pitch_target = msg.data*180/3.14
        
    def callback_joy(self, msg):
        self.button = msg.buttons
        self.axes = msg.axes

    def callback_press(self, msg):
        self.pressure = msg.fluid_pressure / 100000

    def callback_depth(self, msg):
        self.depth = -msg.data

    def callback_target_depth(self, msg):
        self.depth_target = msg.data

    def callback_battery_state(self, msg):
        self.battery_state = round(msg.voltage, 2)

    def callback_target_distance(self, msg):
        self.distance_target = round(msg.data, 2)

    def callback_distance(self, msg):
        self.distance = round(msg.data, 2)

    def callback_tracking_buoy(self, msg):
        self.tracking_buoy = msg.data

    def callback_tracking_with_heading_mode(self, msg):
        self.tracking_with_heading_mode = msg.data

    def callback_tracking_with_distance_mode(self, msg):
        self.tracking_with_distance_mode = msg.data

    def callback_tracking_wihtout_depth_control(self, msg):
        self.tracking_wihtout_depth_control = msg.data

    def callback_heading_offset(self, msg):
        self.heading_offset = msg.data
        
    def callback_override(self, msg):
        self.commands = msg.channels

    def callback_record(self, msg):
        self.record = msg.data

    def callback_choice_record(self, msg):
        self.record_step = msg.data[0]
        self.record_choice_record = msg.data[1]

    def callback_joy_switch(self, data):
        self.joy_active = data.data
        if (self.joy_active == self.ID0):
            self.piloted = True
        else:
            self.piloted = False
    
    def callback_light_pwm(self, data):
        self.light_pwm = data.data
    
    def callback_ping1d(self, data):
        self.distance_ping = data.data[0]
        self.confidence_ping = data.data[1]
        
    def callback_mode_actif_ROV(self, data):
        if data.data[0] == 1:
            self.modeB = True
        else:
            self.modeB = False
        self.modeX = data.data[1]
        
    def callback_save(self,data):
        self.record2 = data.data  
              
    def callback_program_B_name(self,data):
        self.program_B_name = data.data    
        
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
        Phi_rad, Theta_rad, Psy_rad = Rotation.from_quat([orientq[1], orientq[2], orientq[3],   orientq[0]]).as_euler("xyz") # Roulis, Tangage, Lacet     
        self.Phi, self.Theta, self.Psy = Phi_rad*180/3.14 , Theta_rad*180/3.14, Psy_rad*180/3.14    
                  
    def listener(self):
        self.state_sub            = self.create_subscription(State,               self.ns+'/sensors/state',                       self.callback_armed,                        self.qos_profile)
        self.battery_sub          = self.create_subscription(BatteryState,        self.ns+'/sensors/battery',                     self.callback_battery_state,                self.qos_profile)
        self.imu_sub              = self.create_subscription(Imu,                 self.ns+"/sensors/imu/data",                    self.callback_imu,                          self.qos_profile)
        self.press_sub            = self.create_subscription(FluidPressure,       self.ns+'/sensors/imu/static_pressure',         self.callback_press,                        self.qos_profile)
        self.compass_sub          = self.create_subscription(Float64,             self.ns+'/sensors/global_position/compass_hdg', self.callback_compass,                      self.qos_profile)
        self.depth_sub            = self.create_subscription(Float64,             self.ns+'/sensors/global_position/rel_alt',     self.callback_depth,                        self.qos_profile)
        self.override_rc_sub      = self.create_subscription(OverrideRCIn,        self.ns+"/sensors/rc/override",                 self.callback_override,                     self.queue_listener)
        self.msg_ping1d_sub       = self.create_subscription(Float32MultiArray,   self.ns+'/msg_ping1d',                          self.callback_ping1d,                       self.queue_listener)
        self.mode_actif_ROV_sub   = self.create_subscription(Float32MultiArray,   self.ns+"/mode_actif_ROV",                      self.callback_mode_actif_ROV,               self.queue_listener)
        self.choice_record_sub    = self.create_subscription(Float32MultiArray,   self.ns+"/choice_record",                       self.callback_choice_record,                self.queue_listener)
        self.robot_mode_sub       = self.create_subscription(String,              self.ns+'/robot_mode',                          self.callback_mode,                         self.queue_listener)
        self.robot_config_sub     = self.create_subscription(String,              self.ns+'/robot_config',                        self.callback_config,                       self.queue_listener)
        self.program_B_name_sub   = self.create_subscription(String,              self.ns+"/program_B_name",                      self.callback_program_B_name,               self.queue_listener)
        self.target_distance_sub  = self.create_subscription(Float64,             self.ns+'/target_distance',                     self.callback_target_distance,              self.queue_listener)
        self.target_heading_sub   = self.create_subscription(Float64,             self.ns+'/target_heading',                      self.callback_target_heading,               self.queue_listener)
        self.target_depth_sub     = self.create_subscription(Float64,             self.ns+'/target_depth',                        self.callback_target_depth,                 self.queue_listener)
        self.target_roll_sub      = self.create_subscription(Float64,             self.ns+"/target_roll",                         self.callback_target_roll,                  self.queue_listener)
        self.target_pitch_sub     = self.create_subscription(Float64,             self.ns+"/target_pitch",                        self.callback_target_pitch,                 self.queue_listener)
        self.distance_sub         = self.create_subscription(Float64,             self.ns+'/distance',                            self.callback_distance,                     self.queue_listener)
        self.active_light_pwm_sub = self.create_subscription(Float64,             self.ns+"/active_light_pwm",                    self.callback_light_pwm,                    self.queue_listener)
        self.joy_sub              = self.create_subscription(Joy,                 '/joy',                                         self.callback_joy,                          self.queue_listener)
        self.num_joy_control_sub  = self.create_subscription(Float64,             "/num_joy_control",                             self.callback_joy_switch,                   self.queue_listener)

        self.tracking_buoy_sub                   = self.create_subscription(Bool,   self.ns+"/tracking_buoy",                     self.callback_tracking_buoy,                  self.queue_listener)
        self.tracking_with_heading_mode_sub      = self.create_subscription(Bool,   self.ns+"/tracking_with_heading_mode",        self.callback_tracking_with_heading_mode,     self.queue_listener)
        self.tracking_with_distance_mode_sub     = self.create_subscription(Bool,   self.ns+"/tracking_with_distance_mode",       self.callback_tracking_with_distance_mode,    self.queue_listener)
        self.tracking_wihtout_depth_control_sub  = self.create_subscription(Bool,   self.ns+"/tracking_wihtout_depth_control",    self.callback_tracking_wihtout_depth_control, self.queue_listener)
        self.record_cam_sub                      = self.create_subscription(Bool,   self.ns+"/record_cam",                        self.callback_record,                         self.queue_listener)
        self.enregistrement_en_cours_sub         = self.create_subscription(Bool,   self.ns+"/enregistrement_en_cours",           self.callback_save,                           self.queue_listener)

        

    def display(self, text, color, font=None, dx=0, small_space = False, big_space = False):
        if font is None: font = self.font
        cv2.putText(self.ihm_img, text, (self.x + dx, self.y), font, self.font_scale, color, self.font_thickness)
        if small_space: self.small_space()
        if big_space:   self.big_space()

    def display_if(self, text1, color1, condition, text2, color2, font1=None, font2=None, dx=0, small_space = False, big_space = False):
        self.display(text1, color1, font1, dx=dx) if condition else self.display(text2, color2, font2, dx=dx)
        if small_space: self.small_space()
        if big_space:   self.big_space()

    def space(self,eps):  self.y += eps
    def big_space(self):   self.space(self.big_eps)
    def small_space(self): self.space(self.eps)
    

    def display_joysticks(self):
        cv2.circle(self.ihm_img, (int(2 * self.R), self.y), self.R, self.grey, 5)
        cv2.circle(self.ihm_img, (int(6 * self.R), self.y), self.R, self.grey, 5)
        cv2.circle(self.ihm_img, (int(2 * self.R - self.axes[0] * self.R), int(self.y - self.axes[1] * self.R)), self.r, self.blue, -1)
        cv2.circle(self.ihm_img, (int(6 * self.R - self.axes[3] * self.R), int(self.y - self.axes[4] * self.R)), self.r, self.blue, -1)

    def run(self):
        # Reinitialisation
        self.ihm_img = np.zeros(self.ihm_size, np.uint8)
        self.x, self.y = 20, 30
        if (self.detected == 1)&(self.first_detection == 0):
            self.first_detection = 1

        # ---- ROBOT STATE ----
        val_light = (self.light_pwm-1000)/10 + 10                   

        self.display("--- " + self.ROV_name + ", n." + str(self.ID) +" ---", self.white, self.big_font, small_space= True)
        self.display_if("ROV piloted (L3/R3)", self.green, self.piloted, "ROV not piloted (n."+ str(int(self.joy_active)) +" ON) ", self.red, small_space= True) 
        self.display_if("Armed (start)",       self.red, self.armed, "Disarmed  (start)", self.grey, small_space= True)
        self.display_if("BOST ON",             self.purple, self.axes[5] < 0, "Boost (hold RT)", self.grey, small_space= True)
        self.display_if("Battery State : " + str(self.battery_state) + "V", self.green, self.battery_state > self.min_battery_state, "Battery State :"+ str(self.battery_state) + "V", self.red, small_space= True)
        self.display_if("Light level (back): " + str(val_light) + "%", self.green, 100 > val_light, "Light level : " + str(val_light) + "%", self.red, small_space= True)                 
        
        # Recording
        if not( (self.record_step == 1)|(self.record_step == 2)):
            if not(self.tracking_buoy): # cas classique
                self.display_if("Recording (Y): ON", self.red, (self.record)|(self.record2), "Recording (Y): OFF", self.grey)
        
        else: # sinon, on propose les choix
            self.small_space()
            self.display("Record : SELECT ONE CHOICE:", self.red, self.big_font, small_space= True)
            self.display_if(" <- : data+cam ", self.green, (self.record_choice_record == 1), " <- : data+cam ", self.grey, self.big_font)
            self.display_if(" -> : data without cam ", self.green, (self.record_choice_record == 2), " -> : data without cam", self.grey, self.big_font, dx = 300, small_space= True)
            self.display('AND press "Y" again', self.grey, self.big_font)
            self.space(10*self.eps)
            
        self.small_space()                                            
        
        # info camera
        self.display("Move cam: (arrows)", self.grey)
        self.small_space()  

        if (self.modeX == 0)&(self.robot_config == "heavy"): self.display( 'Roll: ' + str(int(self.Phi))    + 'deg (LB/RB)', self.blue)
        else:                                                self.display( 'Roll: ' + str(int(self.Phi))    + 'deg        ', self.blue)
        if self.robot_config == "heavy":                     self.display( "/ "+ str(int(self.roll_target)) + "deg", self.green, dx = 250 )
        self.small_space() 

        if (self.modeX == 1)&(self.robot_config == "heavy"): self.display( 'Pitch: ' + str(int(self.Theta))  + 'deg (LB/RB)', self.blue)
        else:                                                self.display( 'Pitch: ' + str(int(self.Theta))  + 'deg        ', self.blue)
        if self.robot_config == "heavy":                     self.display( "/ "+ str(int(self.pitch_target)) + "deg", self.green, dx = 250 )
        self.small_space()           
        
        self.space(-10*self.eps)
        dx0 = 310  + 80
        
        
        # ---- DISPLAY RIGHT ----
        self.display("--- TELEMETRY ---",                                                                                       self.white, self.big_font, dx = dx0, small_space=True)
        self.display("Altitude: " + str(round(self.distance_ping, 4))+ "m (conf.: "+ str(round(self.confidence_ping)) + "%)" ,  self.blue,  dx = dx0, small_space=True)
        self.display("Depth: "     + str(self.depth) + "m",                                                                     self.blue,  dx = dx0 )
        self.display("Target: "    + str(self.depth_target) + "m",                                                              self.green, dx = dx0 + 280, small_space=True)
        self.display("Heading: "   + str(self.heading) + "deg",                                                                 self.blue,  dx = dx0 )
        self.display_if("Target: " + str(int(self.heading_target))+ "deg",                                                      self.green, True, "-", self.green, dx=280+dx0, small_space=True)
        
        if self.tracking_buoy: self.display("Distance from target: " + str(self.distance),                                      self.blue,  dx = dx0, big_space=True )
        self.small_space()
        self.display("---- ROV ----",                                                                                           self.white, self.big_font, dx = dx0, small_space=True)
        self.display_if("Tracking ON (A)",                                                                                      self.green, self.tracking_buoy, "No tracking (A)", self.grey, dx = dx0 )
        if self.tracking_buoy: self.display_if("/ Depth control (Y)",                                                           self.green, not(self.tracking_wihtout_depth_control), "/ No depth control (Y)", self.grey, dx = dx0 + 200 )
            
        self.small_space()
        if not(self.tracking_buoy):
            if (self.modeX == 0)&(self.robot_config == "heavy"):
                self.display("(X) control :", self.grey,  dx = dx0)
                self.display(" roll",         self.green, dx = dx0 + 150)
                self.display("/ ",            self.grey,  dx = dx0 + 200)
                self.display("pitch ",        self.grey,  dx = dx0 + 210)
                self.display("/ ",            self.grey,  dx = dx0 + 270)
                self.display(" rep. global",  self.grey,  dx = dx0 + 280)

            elif (self.modeX == 1)&(self.robot_config == "heavy"):
                self.display("(X) control :", self.grey,  dx = dx0)
                self.display(" roll",         self.grey,  dx = dx0 + 150)
                self.display("/ ",            self.grey,  dx = dx0 + 200)
                self.display("pitch ",        self.green, dx = dx0 + 210)
                self.display("/ ",            self.grey,  dx = dx0 + 270)
                self.display(" rep. global",  self.grey,  dx = dx0 + 280)
            
            elif (self.modeX == 2)&(self.robot_config == "heavy"):
                self.display("(X) control :", self.grey,  dx = dx0)
                self.display(" roll",         self.grey,  dx = dx0 + 150)
                self.display("/ ",            self.grey,  dx = dx0 + 200)
                self.display("pitch ",        self.grey,  dx = dx0 + 210)
                self.display("/ ",            self.grey,  dx = dx0 + 270)
                self.display(" rep. global",  self.green, dx = dx0 + 280)

            self.small_space()
            self.display_if("Program (B) : "+self.program_B_name, self.green, self.modeB, "No program (B): "+self.program_B_name, self.grey, dx = dx0, small_space=True)
            
        if self.tracking_buoy: # only possible is tracking activated
            self.display_if("Heading offset (B): " + str(self.heading_offset) + "deg (<-/->)", self.green, self.tracking_with_heading_mode, "No heading tracked (B) (" + str(self.heading_offset) + "deg)", self.grey, dx = dx0, small_space=True )
            self.display_if("Desired distance (X):"+ str(self.distance_target)+ '(LB/RB)', self.green, self.tracking_with_distance_mode, "No distance tracked (X)("+ str(self.distance_target) + ")", self.grey, dx = dx0, small_space=True )
        else:
            if (self.first_detection == 1)&(self.detected == 0):
                self.display_if("Target lost", self.red, self.detected, "Target lost", self.red, dx = dx0, small_space=True )
            else:
                self.small_space()
                self.small_space()

        cv2.imshow("Telemetry ", self.ihm_img) 
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node_ihm = IHM()
    rclpy.spin(node_ihm)
    node_ihm.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    

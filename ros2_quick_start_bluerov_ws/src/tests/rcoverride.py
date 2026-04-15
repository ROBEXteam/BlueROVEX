#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function 
import time
import math
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink1
#from pymavlink.dialects.v20 import ardupilotmega as mavlink2

# DO NOT USE THIS CODE DIRECTLY ON A REAL ROBOT WITHOUT CHECKING WHAT IT MIGHT DO!

# See https://mavlink.io/en/messages/common.html
# You might be interested in setting e.g. RC10_OPTION to 46 (RC Override Enable) to be able to disable overrides using a switch on the radio (since firmware 3.6, note that this might not work with SITL), and/or setting a timeout using RC_OVERRIDE_TIME parameter.
# Note also you can set e.g. SERVO1_FUNCTION to 51, SERVO2_FUNCTION to 52 to get RCIN1, RCIN2 on corresponding servo output pins of the autopilot...

#
# Please customize mavutil.mavlink_connection() parameters to your setup!
#
# Default mavutil.mavlink_connection() parameters should be source_system=255, source_component=0. If the robot appears to ignore some messages, please check if source_system matches ArduRover parameters SYSID_MYGCS (might be removed from recent versions) or MAV_GCS_SYSID and MAV_GCS_SYSID_HI, see https://ardupilot.org/rover/docs/parameters.html#mav-gcs-sysid-my-ground-station-number...
#autopilot = mavutil.mavlink_connection("COM12", 115200)
#autopilot = mavutil.mavlink_connection('tcp:127.0.0.1:5760')
autopilot = mavutil.mavlink_connection('udp:192.168.2.1:14571')
autopilot.wait_heartbeat()
print("Heartbeat from (system %u, component %u) to (system %u, component %u)" % (autopilot.target_system, autopilot.target_component, autopilot.mav.srcSystem, autopilot.mav.srcComponent))
n = 4; dt = 0.1 # To send repeatedly the messages in case of unreliable network
# Neutral RC...
for i in range(0,n):
    autopilot.mav.rc_channels_override_send(autopilot.target_system, autopilot.target_component, 1500, 1500, 1500, 1500, 0, 0, 0, 0)
    time.sleep(dt)
# Arm
print("Arming")
for i in range(0,n):
    autopilot.mav.command_long_send(autopilot.target_system, autopilot.target_component, mavlink1.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
    time.sleep(dt)
time.sleep(3)
# Manual (0) mode...
# See enum control_mode_t in https://github.com/ArduPilot/ardupilot/blob/autopilot/Rover/defines.h, https://github.com/ArduPilot/ardupilot/blob/master/Rover/mode.h
for i in range(0,n):
    autopilot.mav.set_mode_send(autopilot.target_system, mavlink1.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)
    time.sleep(dt)
time.sleep(1)
# Move
print("Forward")
t = 4
for i in range(0,int(t/dt)):
    autopilot.mav.rc_channels_override_send(autopilot.target_system, autopilot.target_component, 1500, 1500, 1750, 1500, 0, 0, 0, 0)
    time.sleep(dt)
print("Turn")
t = 4
for i in range(0,int(t/dt)):
    autopilot.mav.rc_channels_override_send(autopilot.target_system, autopilot.target_component, 1400, 1500, 1750, 1500, 0, 0, 0, 0)
    time.sleep(dt)
print("Stop")
for i in range(0,n):
    autopilot.mav.rc_channels_override_send(autopilot.target_system, autopilot.target_component, 1500, 1500, 1500, 1500, 0, 0, 0, 0)
    time.sleep(dt)
time.sleep(2)
autopilot.close()
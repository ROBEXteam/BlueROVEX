

git : 
https://github.com/cviel/Quick_start_bluerov/tree/main/quick_start_bluerov_ros2


-----------------------------

1) installer QGroundControl :

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

-> creer le network pour s'y connecter


2) installer ROS2

https://docs.ros.org/en/humble/Installation.html


3) installer node_joy
sudo apt update
sudo apt install ros-humble-joy

test si fonctionne :
ros2 run joy joy_node
ros2 topic echo /joy


4) Python environment

Following modules are required:

- numpy
- opencv-python (version 4.2.0) 
- opencv-contrib-python
- pyautogui
- imutils
- sockets
- vidgear
- brping
- sshpass : sudo apt-get install sshpass
- ffmpeg

### Autre commentaire :



penser a mettre dans le .bashrc  le source "install/setup.bash"
--> "source ~/quick_start_bluerov_ros2/install/setup.bash"


penser à faire en sorte que TOUS les fichiers soient des executables (chmod +x)


-------------

pour occulus :

https://github.com/godardma/oculus_ros2






ros2 run bluerov_control camera_node --ros-args -r __ns:=/castor -p port:=5600 -p publish_raw:=true -p publisher_compressed:=false
ros2 run bluerov_control camera_node --ros-args -r __ns:=/pollux -p port:=5622 -p publish_raw:=true
ros2 launch bluerov_control main.launch.py rovs:=inky

ros2 run aruco_lib vision_treatment


CALIBRATION : 
ros2 run camera_calibration cameracalibrator \
  --size 8x5 --square 0.025 \
  image:=/image


# Launch bluerov

ros2 launch bluerov_control main.launch.py rovs:="castor" (change castor by inky or other rovs name)
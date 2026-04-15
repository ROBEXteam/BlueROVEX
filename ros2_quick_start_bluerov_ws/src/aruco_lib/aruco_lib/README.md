Nodes ROS2 sur mon PC : control, guidance, localization, actuation
Nodes ROS2 depuis la simulation : 


Node control :
- récupère les données du guidage depuis le topic /guidance_simu
- publie sur le topic /control_simu une liste avec les controles à donner selon les 6 degrés de liberté

Node actuation :
- récupère les données du controle sur les 6 DOF depuis le topic /control_simu
- publie sur le topic /actuation_simu une liste avec les forces à exercer sur les 8 moteurs du BlueROV en configuration Heavy

Node worldlocalization :
- récupère les données des capteurs sur les topics /gnss_simu /pressure_simu /imu_simu /camera_localization_target_simu /sonar_localization_target_simu
- publie sur le topic /worldlocalization_simu une liste avec la position dans le repère terrestre (lat,lon,profondeur) et son orientation (/phi, /theta, /psi)

Node targetLocalization :
- récupère les données des capteurs de positionnement de la cible depuis les topics /camera_localization_target_simu /sonar_localization_target_simu
- publie sur le topic /target_localization_simu une liste avec la fusion des données capteur

Node depthGuidance :
- récupère la consigne de profondeur depuis le topic /planning (flottant) et la profondeur depuis /pressure_simu
- publie l'écart entre la profondeur consigne et actuelle sur /depth_guidance_simu

 
Node planning : (pas fait encore - envoyer une commande direct)
- publie une profondeur désirée par l'opérateur sur le topic /planning_simu c'est un flottant



cd Documents/ros2_ws/; source install/setup.bash; clear

ros2 topic pub -r 1 /planning_simu std_msgs/Float32MultiArray "{data: [2.5, 2.0, 3.0, 0.0, 0.0, 3.14]}"

ros2 launch bluerov_simu depth_guidance.launch.py

ros2 topic echo /control_simu

ros2 run plotjuggler plotjuggler --layout /home/tbelier/Documents/ros2_ws/test/layoutPlotJuggler3.xml


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):

    rovs_param = LaunchConfiguration("rovs").perform(context)
    
    config = os.path.join( get_package_share_directory("bluerov_control"), "config", "parametre_bluerovs.yaml")
    configSBG = os.path.join(
		get_package_share_directory('sbg_driver'),
		'config',
		'sbg_device_uart_default.yaml'
	)
    if rovs_param == "": rovs = ["inky"] # si vide -> défaut
    else: rovs = rovs_param.split(",")

    if rovs_param == "": 
        print("Aucun ROV spécifié, utilisation de 'inky' par défaut.", flush=True)
        print("Pour spécifier les ROVs, utilisez l'argument 'rovs' avec une liste de noms séparés par des virgules, par exemple: 'rovs:=inky,blinky'", flush=True)

    nodes = []
    for i, rov in enumerate(rovs, start=1):  # start=1 pour numéroter les ROVs à partir de 1
        nodes.append(Node(package="bluerov_control",            executable="mavlink_node",    namespace=rov, name=f"mavlink_{rov}", emulate_tty= True, output="screen", parameters=[config]))
        nodes.append(Node(package="bluerov_control",            executable="control_target_node",    namespace=rov, name=f"control_target_{rov}", emulate_tty=True, output="screen", parameters=[config,{"rov_id": i}]))    
        nodes.append(Node(package="bluerov_control",            executable="camera_node",     namespace=rov, name=f"camera_{rov}", emulate_tty= True, output="screen", parameters=[config]))

    nodes.append(Node(package="joy",             executable="joy_node",          name="joy_node",                                        output="screen"))
    nodes.append(Node(package="bluerov_control", executable="rov_selector_node", name=f"rov_selector", parameters=[{"Nrov": len(rovs)}], output="screen"))
    
    return nodes


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("rovs", default_value="inky", description="Liste des ROVs séparés par des virgules"),
        OpaqueFunction(function=launch_setup),])
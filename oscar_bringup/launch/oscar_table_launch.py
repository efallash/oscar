import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import (OnProcessStart, OnProcessExit)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable


from launch_ros.actions import Node

import xacro
#from moveit_config_utils import MoveItConfigsBuilder



def generate_launch_description():

    #Get Directories
    pkg_name='oscar_description'
    package_path= os.path.join(get_package_share_directory(pkg_name))
    xacro_file=os.path.join(package_path,'urdf', 'oscar.urdf.xacro')


    #Set gazebo resource path
    thor_share_path= os.path.join(get_package_prefix('thor_description'), 'share')
    gripper_share_path= os.path.join(get_package_prefix('gripper_description'),'share')
    oscar_share_path=os.path.join(get_package_prefix('oscar_description'), 'share')
    oscar_gazebo_share_path=os.path.join(get_package_prefix('oscar_gazebo'), 'share', 'oscar_gazebo', 'models')

    set_env_vars_resources_thor = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', thor_share_path)
    set_env_vars_resources_gripper = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', gripper_share_path)
    set_env_vars_resources_oscar = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', oscar_share_path)
    set_env_vars_resources_oscar_gazebo = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH', oscar_gazebo_share_path)
    



    #Launch Gazebo 
    gazebo_pkg_dir=get_package_share_directory('gazebo_ros')
    world = os.path.join(
        get_package_share_directory('oscar_gazebo'),
        'worlds',
        'table.world'
    )
    gzserver_args={'world': world, 'verbose': 'false', 'server_required':'true'}.items()
    gzclient_args={}.items()
    #Gazebo client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments=gzserver_args
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments=gzclient_args
    )


    
    #Parse xacro file and save to parameter 'robot_description'
    robot_description= xacro.parse(open(xacro_file))
    xacro.process_doc(robot_description)
    params={'robot_description': robot_description.toxml()}

    #Robot State Publisher
    node_robot_state_publisher= Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    """
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )
    """


    #Spawn Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'oscar'],
        output='screen'
    )

    """
    joint_state_event=RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity, 
                                    on_exit=[load_joint_state_broadcaster]))
    
    load_controllers_event=RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_joint_state_broadcaster, 
                                    on_exit=[load_arm_controller,load_gripper_controller]))
    
    """

    ld= LaunchDescription()
    ld.add_action(set_env_vars_resources_gripper)
    ld.add_action(set_env_vars_resources_thor)
    ld.add_action(set_env_vars_resources_oscar)
    ld.add_action(set_env_vars_resources_oscar_gazebo)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)


    #ld.add_action(joint_state_event)
    #ld.add_action(load_controllers_event)
    ld.add_action(node_robot_state_publisher)

    ld.add_action(spawn_entity)

    return ld
import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    examples_dir = FindPackageShare('dorna_example').find('dorna_example')

    r1 = make_dorna_simulation("r1",  os.path.join(examples_dir, 'poses', 'r1_joint_poses.csv'))
    r2 = make_dorna_simulation("r2",  os.path.join(examples_dir, 'poses', 'r2_joint_poses.csv'))

    sm_parameters = {
        "active_transforms": os.path.join(examples_dir, 'transforms', 'active_transforms.json'),
        "static_transforms": os.path.join(examples_dir, 'transforms', 'static_transforms.json')
    }

    scene_manipulation_service = launch_ros.actions.Node(
                package='ros2_scene_manipulation',
                executable='service_main',
                output='screen',
                parameters=[sm_parameters]
                )

    camera_node = launch_ros.actions.Node(
                package='camera_driver',
                executable='camera_simulator',
                namespace='/camera',
                output='screen',
                )

    control_box = launch_ros.actions.Node(
                package='control_box_driver',
                executable='control_box_simulator',
                namespace='/control_box',
                output='screen',
                )

    sp_ui = launch_ros.actions.Node(
                package='sp_ui',
                executable='sp_ui',
                namespace='/',
                output='screen',
                )
    sp_operator = launch_ros.actions.Node(
                package='sp_operator',
                executable='sp_operator',
                namespace='/sp_operator/op1',
                output='screen',
                )

    sp = launch_ros.actions.Node(
                package='cylinders',
                executable='cylinders',
                output='screen',
                )

    rviz_config_file = os.path.join(examples_dir, 'config', 'dorna.rviz')
    launch_rviz = launch.actions.DeclareLaunchArgument(name="rviz", default_value="False", description="Launch RViz?")
    rviz_cond = launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration("rviz"))
    rviz = launch_ros.actions.Node(package='rviz2', executable='rviz2',
                                   arguments=['-d', rviz_config_file],
                                   output='screen', condition = rviz_cond)

    rviz_nodes = [launch_rviz, rviz]
    nodes = [scene_manipulation_service, camera_node, control_box, sp_ui, sp_operator, sp]
    return launch.LaunchDescription(r1 + r2 + nodes + rviz_nodes)



def make_dorna_simulation(name, poses_file):
    dorna2_share = FindPackageShare('dorna2').find('dorna2')
    urdf = os.path.join(dorna2_share, 'urdf', 'dorna.urdf')
    with open(urdf, 'r') as infp:
       robot_desc = infp.read()

    robot_state_publisher_params = {
        "robot_description": robot_desc,
    }
    rsp_node = launch_ros.actions.Node(package='robot_state_publisher',
                                executable='robot_state_publisher',
                                namespace='dorna/' + name,
                                output='screen',
                                parameters=[robot_state_publisher_params]
                                )

    simulation_parameters = {
        "saved_poses_file": poses_file,
        "joint_names": [
            "dorna_axis_1_joint",
            "dorna_axis_2_joint",
            "dorna_axis_3_joint",
            "dorna_axis_4_joint",
            "dorna_axis_5_joint",
        ],
        "max_joint_speed": [45, 45, 45, 45, 45],  # deg/sec
        "joint_state_timer_period": 0.05,
        "robot_state_initial_pose_name": "pre_take"
    }
    sim_node = launch_ros.actions.Node(package='robot_simulator',
                 executable='robot_simulator',
                 namespace='dorna/'+name,
                 output='screen',
                 parameters=[simulation_parameters]
                 )

    gui_parameters = {
        "saved_poses_file": poses_file,
        "joint_names": [
            "dorna_axis_1_joint",
            "dorna_axis_2_joint",
            "dorna_axis_3_joint",
            "dorna_axis_4_joint",
            "dorna_axis_5_joint",
        ],
        "joint_limit_max": [
            175,
            160,
            130,
            180,
            180,
        ],
        "joint_limit_min": [
            -175,
            -175,
            -130,
            -180,
            -180,
        ]
    }
    gui_node = launch_ros.actions.Node(
                package='robot_gui',
                executable='gui',
                namespace='dorna/'+name,
                output='screen',
                parameters=[gui_parameters]
                )

    return [rsp_node, sim_node, gui_node]

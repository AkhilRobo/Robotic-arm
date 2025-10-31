import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_panda = get_package_share_directory('panda_description')

    # 1️⃣  Set Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_panda]
    )

    # 2️⃣  Load robot description
    robot_description = Command([
        'xacro ',
        os.path.join(pkg_panda, 'urdf', 'panda.urdf.xacro')
    ])

    # 3️⃣  Start controller_manager (ros2_control_node)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg_panda, 'config', 'ros2_controllers.yaml'),
            {'robot_description': robot_description,
             'use_sim_time': True}
        ],
        output='screen',
        emulate_tty=True
    )

    # 4️⃣  Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    # 5️⃣  Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # 6️⃣  Spawn robot into Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'panda', '-topic', '/robot_description'],
        output='screen'
    )

    # 7️⃣  Spawner nodes for controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    eef_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['eef_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Ensure controllers load after robot is spawned
    delayed_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )
    delayed_eef_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[eef_controller_spawner]
        )
    )

    # 8️⃣  Camera bridges (same as yours)
    color_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='color_camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/color_camera@sensor_msgs/msg/Image[ignition.msgs.Image'],
    )

    depth_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
    )

    # ✅ Final Launch Description
    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gazebo)
    ld.add_action(controller_manager)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity)
    ld.add_action(delayed_joint_state)
    ld.add_action(delayed_arm_controller)
    ld.add_action(delayed_eef_controller)
    ld.add_action(color_camera_bridge)
    ld.add_action(depth_camera_bridge)

    return ld

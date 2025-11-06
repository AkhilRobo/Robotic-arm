
import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,RegisterEventHandler,DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import (
    Command, 
    PathJoinSubstitution, 
    LaunchConfiguration 
)

def generate_launch_description():

    ld = LaunchDescription()
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_path = 'urdf/PoleSensor.urdf.xacro'
    
    robot_description = PathJoinSubstitution([
        get_package_share_directory('panda_description'),	
        xacro_path
    ])
    
    set_prime_offload = SetEnvironmentVariable(
    name='__NV_PRIME_RENDER_OFFLOAD',
    value='1'
)

    set_glx_vendor = SetEnvironmentVariable(
    name='__GLX_VENDOR_LIBRARY_NAME',
    value='nvidia'
)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',        
        parameters=[{
            'robot_description':Command(['xacro ', robot_description] ),
            'use_sim_time': use_sim_time
        }]
    )



    workspace_path = os.environ.get('COLCON_PREFIX_PATH') or os.environ.get('AMENT_PREFIX_PATH')

    pkg_panda_description = os.path.join(workspace_path, "panda_description", "share")

    gz_models_path = "/home/akhil/.ignition/models"

    gz_resource_path = SetEnvironmentVariable(
    name="GZ_SIM_RESOURCE_PATH",
    value=[f"{pkg_panda_description}:{gz_models_path}"]
)

    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'panda', '-topic', '/robot_description' ,
                            '-x', '0.438782',
                            '-y', '0.134319',
                            '-z', '0.775935'], 
                            output='screen')

    
    

    sdf_file_path = os.path.join(
        FindPackageShare('panda_description').find('panda_description'),
        'world',
        'Mainworld.sdf'
        
    )
    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {sdf_file_path}',
            'use_sim_time': use_sim_time 

        }.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/panda/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        parameters=[{'use_sim_time': use_sim_time}], 
    )

    depth_cam_data2cam_link_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_tf',
    output='screen',
    arguments=[
        '0', '0', '0',
        '0', '0', '0',         
        'camera_link_optical',               
        'panda/link_1/my_rgbd_camera'
    ]
)



    ld.add_action(use_sim_time_arg)
    ld.add_action(set_prime_offload)
    ld.add_action(set_glx_vendor)
    ld.add_action( gz_resource_path )
    ld.add_action( ignition_gazebo_node )

    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )



    
    ld.add_action( gz_bridge_node )
    ld.add_action( depth_cam_data2cam_link_tf )


    ld.add_action(clock_bridge) 


    return ld

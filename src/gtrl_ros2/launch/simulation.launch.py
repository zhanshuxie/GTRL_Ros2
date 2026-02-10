import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取包路径
    pkg_gtrl = get_package_share_directory('gtrl_ros2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # 2. 定义文件路径
    # 注意：这里对应你 urdf 文件夹下的结构
    xacro_file = os.path.join(pkg_gtrl, 'urdf', 'xacro', 'bringup', 'bringup_scout.xacro')
    world_file = os.path.join(pkg_gtrl, 'worlds', 'RRC1.world') # 确保你有这个world文件
    rviz_config = os.path.join(pkg_gtrl, 'rviz', 'scout_ros2.rviz')   # 确保你有这个rviz文件

    # 3. 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x', default='-1.5')
    y_pose = LaunchConfiguration('y', default='0.0')

    # 4. 解析 Xacro 文件 (这是关键步骤)
    # Command 会执行 xacro 命令并将结果作为 robot_description 参数
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # 将解析后的 XML 包装在字典中
    robot_description = {'robot_description': robot_description}

    # 5. 定义节点

    # (A) 启动 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': 'false'}.items(),
    )

    # (B) 发布机器人状态 (Robot State Publisher)
    # 它会将 URDF 发布到 /robot_description 话题，并计算 TF
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # # (C) 关节状态发布 (Joint State Publisher)
    # # 如果 Gazebo 插件没有正确发布 joint_states，这个节点可以作为补充
    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )

    # (D) 在 Gazebo 中生成机器人 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', # 从话题读取 XML
            '-entity', 'scout',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.2' #稍微抬高一点防止卡地里
        ],
        output='screen'
    )

    # (E) RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='-1.5'),
        DeclareLaunchArgument('y', default_value='0.0'),
        
        # 1. 先启动 Gazebo
        gazebo,
        
        # 2. 延迟 3秒 后启动 Robot State Publisher (等待 /clock 信号)
        TimerAction(
            period=3.0, 
            actions=[node_robot_state_publisher]
        ),
        
        # 3. 延迟 3秒 后生成机器人 (Spawn)
        TimerAction(
            period=3.0, 
            actions=[spawn_entity]
        ),

        # 4. 延迟 5秒 后启动 RViz2 (确保 TF 树已经建立)
        TimerAction(
            period=5.0, 
            actions=[node_rviz]
        ),
    ])
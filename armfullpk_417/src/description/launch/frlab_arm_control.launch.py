import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_desc = get_package_share_directory('description')

    urdf_file        = os.path.join(pkg_desc, 'urdf',   'ur5e_gripper.urdf')
    arm_only_file    = os.path.join(pkg_desc, 'urdf',   'ur5e_arm_only.urdf')
    controllers_yaml = os.path.join(pkg_desc, 'config', 'controllers.yaml')
    rviz_config      = os.path.join(pkg_desc, 'config', 'rviz.rviz')

    use_fake_hardware = LaunchConfiguration('use_fake_hardware', default='true')

    with open(urdf_file, 'r') as f:
        robot_description_real = f.read()

    with open(arm_only_file, 'r') as f:
        kinematics_description = f.read()

    # spawner에 kinematics_description 전달용 임시 YAML
    _kin_yaml = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(
        {
            'dls_controller': {
                'ros__parameters': {
                    'kinematics_description': kinematics_description,
                    # arm_only URDF uses tool0 as the 6-DOF IK end-effector frame.
                    'end_effector_name': 'tool0',
                }
            }
        },
        _kin_yaml)
    _kin_yaml.flush()
    kin_yaml_path = _kin_yaml.name

    # fake 모드: 실제 하드웨어 플러그인 → mock으로 교체
    robot_description_fake = robot_description_real.replace(
        '<plugin>frlab_arm_hardware/FrlabArmHardware</plugin>',
        '<plugin>mock_components/GenericSystem</plugin>'
    ).replace(
        '<param name="can_interface">can0</param>', ''
    ).replace(
        '<param name="default_velocity">0.5</param>', ''
    ).replace(
        '<param name="read_deadline_ms">4</param>', ''
    ).replace(
        '<param name="read_poll_timeout_ms">1</param>', ''
    ).replace(
        '<param name="perf_log_every_n_cycles">0</param>', ''
    )

    robot_state_pub_real = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_real}],
        condition=UnlessCondition(use_fake_hardware),
    )

    robot_state_pub_fake = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_fake}],
        condition=IfCondition(use_fake_hardware),
    )

    controller_manager_real = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_real},
            controllers_yaml,
        ],
        output='screen',
        condition=UnlessCondition(use_fake_hardware),
    )

    controller_manager_fake = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description_fake},
            controllers_yaml,
        ],
        output='screen',
        condition=IfCondition(use_fake_hardware),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
        ],
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    dls_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dls_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml,
            '--param-file', kin_yaml_path,
        ],
    )

    dls_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dls_controller_spawner],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='true: mock hardware (no CAN needed) / false: real arm'
        ),

        robot_state_pub_real,
        robot_state_pub_fake,
        controller_manager_real,
        controller_manager_fake,
        delayed_joint_state_broadcaster_spawner,
        dls_controller_after_joint_state_broadcaster,
        rviz,
    ])

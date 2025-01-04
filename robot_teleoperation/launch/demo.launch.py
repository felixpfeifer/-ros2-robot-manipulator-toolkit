from launch import LaunchDescription
from launch_ros import event_handlers
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, LocalSubstitution,
                                  PythonExpression)


def generate_launch_description():

    # Spawn the teleop_controller_node after the MoveGroup node is running
    teleop_node = Node(
        package="robot_teleoperation",
        executable="teleop_controller_node",
        name="teleop_controller_node",
        output="screen",
    )

    demo_node = Node(
        package="robot_teleoperation",
        executable="demo_node",
        name="demo_node",
        output="screen",
    )

    return LaunchDescription(
        [
            teleop_node,
            demo_node
        ]
    )

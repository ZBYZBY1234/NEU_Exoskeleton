import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('sensor_module')
    launch_dir = os.path.join(bringup_dir, 'launch')

    Piezoelectric_cmd = Node(
        package="sensor_module",
        executable="Piezoelectric",
        name="Piezoelectric"
    )

    MPU6050_Left_Thigh_cmd = Node(
        package="sensor_module",
        executable="MPU6050_Thigh",
        name="MPU6050_Thigh"
    )

    MPU6050_Left_Calf_cmd = Node(
        package="sensor_module",
        executable="MPU6050_Calf",
        name="MPU6050_Calf"
    )

    Sensor_cmd = Node(
        package="sensor_module",
        executable="Sensor",
        name="Sensor"
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    # Declare the launch options
    ld.add_action(Piezoelectric_cmd)
    ld.add_action(MPU6050_Left_Thigh_cmd)
    ld.add_action(MPU6050_Left_Calf_cmd)
    ld.add_action(Sensor_cmd)
    return ld

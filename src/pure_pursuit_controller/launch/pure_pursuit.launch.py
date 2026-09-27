#!/usr/bin/env python3
"""Launch file: levanta la simulacion (prius_bringup) y el controlador Pure
Pursuit juntos.

Uso:
    ros2 launch pure_pursuit_controller pure_pursuit.launch.py
    ros2 launch pure_pursuit_controller pure_pursuit.launch.py \
        waypoints_file:=/ruta/absoluta/a/otra_ruta.csv
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    waypoints_file_arg = DeclareLaunchArgument(
        "waypoints_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pure_pursuit_controller"), "config", "waypoints.csv"]
        ),
        description="Ruta al CSV de waypoints a seguir (columnas x, y).",
    )

    # Simulacion: incluye el launch file de prius_bringup que ya usas manualmente.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("prius_bringup"), "launch", "gz_sim.launch.py"]
            )
        )
    )

    # Controlador Pure Pursuit. Parametros ya calibrados contra el model.sdf
    # del prius_hybrid (wheelbase, steering_limit) y la corrida de prueba
    # (v_ref, lookahead_gain, min_lookahead).
    controller = Node(
        package="pure_pursuit_controller",
        executable="pure_pursuit_node",
        name="pure_pursuit_node",
        output="screen",
        parameters=[{
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "wheelbase": 2.7,
            "steering_limit": 0.5,
            "v_ref": 0.605,
            "lookahead_gain": 0.5,
            "min_lookahead": 0.5,
            "goal_tolerance": 0.3,
            "rate_hz": 20.0,
        }],
    )

    # Se retrasa el arranque del controlador para dar tiempo a que Gazebo
    # termine de cargar y el auto quede quieto en su punto de spawn. Si el
    # controlador arranca antes, el primer /odom que reciba podria no
    # corresponder todavia a la pose real y estable del auto.
    delayed_controller = TimerAction(period=8.0, actions=[controller])

    return LaunchDescription([waypoints_file_arg, simulation, delayed_controller])
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("ur3e_on_table"),
            "urdf",
            "ur3e_on_table.urdf.xacro",
        ]),
    ])

    robot_description = {"robot_description": robot_description_content}

    rviz_config = PathJoinSubstitution([
        FindPackageShare("ur3e_on_table"),
        "rviz",
        "ur3e_on_table_default.rviz",
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    pick_and_place_node = Node(
        package="ur3e_on_table",
        executable="pick-and-place",
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        pick_and_place_node,
        rviz_node,
    ])

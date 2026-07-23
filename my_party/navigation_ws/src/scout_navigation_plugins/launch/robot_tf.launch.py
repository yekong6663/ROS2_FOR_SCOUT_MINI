from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # C1965.STEP gives base_link -> MID-360 point origin as
    # (0.2183, 0.0, 0.1190) m. FAST-LIO2 uses the IMU/body frame with
    # t_il=(-0.011, -0.02329, 0.04412) m, so the required inverse static
    # transform is body -> base_link below.
    body_to_base_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_base_link_tf",
        output="screen",
        arguments=[
            "--x", "-0.2293",
            "--y", "-0.02329",
            "--z", "-0.07488",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "body",
            "--child-frame-id", "base_link",
        ],
    )

    return LaunchDescription([body_to_base_link])

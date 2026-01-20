from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 320,
            'image_height': 240,
            'pixel_format': 'yuyv2rgb',
            'framerate': 30.0
        }],
        output='screen'
    )

    perception = Node(
        package='duckie_perception',
        executable='perception_node',
        name='perception_node',
        output='screen'
    )

    safety = Node(
        package='duckie_safety',
        executable='watchdog_node',
        name='motor_watchdog',
        output='screen'
    )

    motor = Node(
        package='duckie_motor',
        executable='motor_node',
        name='motor_node',
        output='screen'
    )

    return LaunchDescription([
        camera,
        perception,
        safety,
        motor
    ])

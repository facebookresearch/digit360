from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import SetEnvironmentVariable, GroupAction, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from digit360.interface.usb.usb import get_digit360_devices
from dataclasses import dataclass
from typing import Dict

@dataclass
class Digit360Descriptor:
    serial: str
    data: str
    audio: str
    ics: str
    base_version: str
    ics_version: str

def generate_d360_group_actions(namespace: str, desc: Digit360Descriptor, image_pub_type: str) -> GroupAction:
    d360_serial_io = Node(
        package="d360",
        executable="serial_io",
        name="serial_io",
        parameters=[
            {"port": desc.data},
            {"reliiable_publisher": False},
        ],
        output="screen",
        respawn=False,
        arguments=["--ros-args", "--log-level", "warn"],
    )


    if image_pub_type == "raw":
        image_pub_node = Node(
            package="d360",
            executable="image_pub_opencv",
            name="image_opencv_publisher_node",
            parameters=[
                {"device": desc.ics}
            ],
            output="screen",
            respawn=False,
        )
    elif image_pub_type == "encoded":
        image_pub_node = Node(
            package="d360",
            executable="image_pub_encode",
            name="image_encode_publisher_node",
            parameters=[
                {"device": desc.ics}
            ],
            output="screen",
            respawn=False,
        )
    else:
        raise ValueError(f"Invalid image_pub_type: {image_pub_type}")

    d360_audio = Node(
        package="d360",
        executable="audio_pub",
        name="audio_publisher_node",
        parameters=[
            {"device_name": desc.audio}
        ],
        respawn=False,
        output="screen",
    )

    return GroupAction([
        PushRosNamespace(namespace=namespace),
        image_pub_node,
        d360_serial_io,
        d360_audio,
    ])


def enumerate_active_d360_actions(context, *args, **kwargs):
    image_pub_type = LaunchConfiguration('image_pub_type').perform(context)

    d360_hw_ifs = {d.serial: d for d in get_digit360_devices()}
    metahand_d360_serials: Dict[str, str] = {f"d360_{i}": serial for i, serial in enumerate(d360_hw_ifs.keys())}
    actions = []
    for finger, serial in metahand_d360_serials.items():
        desc = d360_hw_ifs[serial]
        assert desc.audio != '' and desc.ics != '' and desc.data != '', f"Incomplete digit360 descriptor, power cycle device {desc}"
        actions.append(generate_d360_group_actions(finger, desc, image_pub_type))

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'image_pub_type',
            default_value='encoded',
            description='Select image publisher: raw or encoded'
        ),
        OpaqueFunction(function=enumerate_active_d360_actions)
    ])

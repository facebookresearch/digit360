import os
import yaml
from dataclasses import dataclass
from typing import Dict

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction

from ament_index_python.packages import get_package_share_directory
from digit360.interface.usb.usb import get_digit360_devices


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

    image_pub_node = Node(
        package="d360",
        executable="image_pub_encode" if image_pub_type == "encoded" else "image_pub_opencv",
        name="image_publisher_node",
        parameters=[{"device": desc.ics}],
        output="screen",
        respawn=False,
    )

    d360_audio = Node(
        package="d360",
        executable="audio_pub",
        name="audio_publisher_node",
        parameters=[{"device_name": desc.audio}],
        respawn=False,
        output="screen",
    )

    return GroupAction([
        PushRosNamespace(namespace=namespace),
        image_pub_node,
        d360_serial_io,
        d360_audio,
    ])


def load_hand_config(config_path: str) -> Dict[str, Dict[str, str]]:
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return {
        "right": config.get("right_hand", {}),
        "left": config.get("left_hand", {})
    }


def enumerate_d360_from_yaml(context, *args, **kwargs):
    image_pub_type = LaunchConfiguration('image_pub_type').perform(context)
    config_path = LaunchConfiguration('hand_config').perform(context)

    hand_config = load_hand_config(config_path)
    right_hand = hand_config["right"]
    left_hand = hand_config["left"]

    connected_devices = {d.serial: d for d in get_digit360_devices()}
    num_connected = len(connected_devices)
    print(f"[INFO] Detected {num_connected} connected DIGIT360 device(s)")

    actions = []
    d360_idx = 0

    def launch_hand(hand_name: str, hand_fingers: Dict[str, str]) -> bool:
        nonlocal d360_idx
        if all(serial in connected_devices for serial in hand_fingers.values()):
            print(f"[INFO] Launching {hand_name.upper()} hand:")
            for finger in ["thumb", "index", "middle", "ring"]:
                serial = hand_fingers[finger]
                desc = connected_devices[serial]
                ns = f"d360_{d360_idx}"
                print(f"  - {finger}: {serial} → namespace {ns}")
                actions.append(generate_d360_group_actions(ns, desc, image_pub_type))
                d360_idx += 1
            return True
        else:
            print(f"[WARN] {hand_name.capitalize()} hand incomplete. Required serials not fully connected. Skipping.")
            return False

    launched_any = False
    launched_any |= launch_hand("right", right_hand)
    launched_any |= launch_hand("left", left_hand)

    if not launched_any:
        raise RuntimeError("[ERROR] No complete hands detected among connected devices. Nothing launched.")

    return actions


def generate_launch_description() -> LaunchDescription:
    default_config_path = os.path.join(
        get_package_share_directory('d360'),
        'config',
        'hand_config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_pub_type',
            default_value='encoded',
            description='Select image publisher: raw or encoded'
        ),
        DeclareLaunchArgument(
            'hand_config',
            default_value=default_config_path,
            description='YAML config file with hand layout and serials'
        ),
        OpaqueFunction(function=enumerate_d360_from_yaml)
    ])

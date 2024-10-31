# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

from dataclasses import asdict, dataclass
from typing import Any, Dict, List, Optional
from digit360.interface.digit360 import Digit360Descriptor
import pyudev

def _find_subsystems(context: pyudev.Context, descriptor: Digit360Descriptor) -> None:
    children = context.children
    for child in children:
        if child.subsystem == "tty":
            descriptor.data = child.properties["DEVNAME"]
            descriptor.base_version = child.properties["ID_REVISION"]
        if child.subsystem == "sound":
            if "card" in child.sys_name:
                dev_name = f"hw:{child.sys_name[4:]},0"
                descriptor.audio = dev_name
        if child.subsystem == "video4linux" and int(child.attributes.get("index")) == 0:
            descriptor.ics = child.properties["DEVNAME"]
            descriptor.ics_version = child.properties["ID_REVISION"]


def get_digit360_devices() -> List[Digit360Descriptor]:
    context = pyudev.Context()
    devices = context.list_devices(subsystem="usb", ID_MODEL="DIGIT360_Hub")
    digit360_serials = {d.properties["ID_SERIAL_SHORT"] for d in devices}

    print(
        f"Found {len(digit360_serials)} DIGIT360 devices: {', '.join(digit360_serials)}\n"
    )

    digit360_devices = {
        serial: Digit360Descriptor(serial, "", "", "", "", "")
        for serial in digit360_serials
    }

    for dev in devices:
        serial = dev.properties["ID_SERIAL_SHORT"]
        _find_subsystems(dev, digit360_devices[serial])

    digit360_devices_sorted = sorted(digit360_devices.values(), key=lambda x: x.serial)

    return digit360_devices_sorted


def get_digit360_by_serial(
    devices: List[Digit360Descriptor], serial: str
) -> Optional[Digit360Descriptor]:
    try:
        return next(device for device in devices if device.serial == serial)
    except StopIteration:
        print(
            f"Could not find DIGIT360 with serial number {serial}, \
                check serial number or connection to host!"
        )
        return None


def get_digit360_by_hand(
    devices: List[Digit360Descriptor], hand_cfg: Dict[str, Any]
) -> List[Dict[str, Any]]:
    return [
        {"name": finger, "parameters": cfg.params, "descriptor": digit360}
        for finger, cfg in hand_cfg.items()
        if (digit360 := get_digit360_by_serial(devices, cfg.serial))
    ]


def is_digit360_desc_valid(desc: Digit360Descriptor) -> bool:
    if "" in asdict(desc).values():
        return False
    else:
        return True


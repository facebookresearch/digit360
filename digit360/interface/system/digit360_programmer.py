# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import argparse
import os
import subprocess
import sys
import time
from typing import Any
import proto as d360frame
import pyudev
import serial
from digit360.interface.digit360 import Digit360

DFU_MINIMUM_VERSION = 0.10
SCRIPT_DIR = os.path.dirname(__file__)
DFU_PATH = os.path.join(SCRIPT_DIR, "firmware/dfu-util/")
DFU_UTIL = f"{DFU_PATH}/dfu-util"
D360_FLASH_START_ADDR = 0x08000000

def firmware_update(fw_bin_file: str) -> None:
    fw_cmd = "{0} -d 0483:df11 -a 0 --dfuse-address {1}:leave -R -D {2}".format(
        DFU_UTIL, D360_FLASH_START_ADDR, fw_bin_file
    )
    os.system(fw_cmd)


def check_dfu_util_version() -> None:
    try:
        p = subprocess.Popen(
            [DFU_UTIL, "-V"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        output = p.stdout.read()
    except Exception as e:
        print(f"Cannot find dfu-util, {e}")
        sys.exit()

    _dfu_ver = str(output).split("\\n")[0].split("dfu-util")[1].strip()
    dfu_ver = float(_dfu_ver.split("-")[0])

    if dfu_ver < DFU_MINIMUM_VERSION:
        print(f"dfu-util version greater than {DFU_MINIMUM_VERSION} required!")
        sys.exit()


def is_device_dfu_ready() -> int:
    context = pyudev.Context()
    devices = context.list_devices(subsystem="usb", find_all=True).match_property(
        "ID_VENDOR_ID", "0483"
    )
    return len(list(devices))


def main(args: argparse.Namespace) -> None:
    check_dfu_util_version()

    print(f"Connecting to device at {args.device}...")
    try:
        d360 = Digit360(args.device)
    except Exception as e:
        print(e)
        print(
            "Cannot connect to device, make sure any threads accessing device are closed."
        )
        sys.exit()

    system_msg = d360frame.SystemData()
    system_msg.version_major = 0xFF

    msg = d360frame.Digit360Message(system_data=system_msg)

    d360.send(msg)

    while d360.is_open and not is_device_dfu_ready():
        print("Trying to enter DFU mode...")
        try:
            d360.send(msg)
        except Exception:
            pass
        time.sleep(0.5)

    print("DFU mode ready!")
    firmware_update(args.fw)
    print("Finished downloading firmware!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DIGIT360 Firmware Update Utility")
    parser.add_argument("device", type=str, help="DIGIT360 tty device")
    parser.add_argument("fw", type=str, help="Firmware binary file")

    args = parser.parse_args()
    main(args)

# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import pprint as pp
import typing
from dataclasses import dataclass
import proto as d360frame
import serial
import tyro
from digit360.interface.usb.usb import get_digit360_by_serial, get_digit360_devices
from digit360.interface.digit360 import Digit360

@dataclass
class cfg:
    # device serial
    sn: typing.Optional[str] = None
    # device tty
    tty: typing.Optional[str] = None
    # query connected devices
    list: bool = False

def main() -> None:
    args = tyro.cli(cfg)

    devices = get_digit360_devices()

    if args.sn:
        device = get_digit360_by_serial(devices, args.sn)
        d360 = Digit360(device.data)
    elif args.tty:
        d360 = Digit360(args.tty)
    elif args.list:
        pp.pprint(devices)
        exit()
    else:
        print("You must specify either a serial number or tty!")
        exit()

    assert d360.is_open, "could not open d360 port!"

    system_msg = d360frame.SystemData()
    system_msg.status = 0xF7
    msg = d360frame.Digit360Message(system_data=system_msg)

    print("Sending ident to", args.sn if args.sn else "", args.tty if args.tty else "")
    d360.send(msg)


if __name__ == "__main__":
    main()
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import time
import proto as d360frame
from digit360.interface.digit360 import Digit360

def led_all_off(dev: Digit360) -> None:
    lighting_msg = d360frame.LightingControl()
    lighting_msg.channel = d360frame.LightingChannel.CHANNEL_ALL_RESET
    msg = d360frame.Digit360Message(lighting_control=lighting_msg)

    if dev.is_open:
        dev.send(msg)
        time.sleep(0.001)

def led_set_channel(
    dev: Digit360, channel: int, rgb: tuple
) -> d360frame.Digit360Message:
    lighting_msg = d360frame.LightingControl()
    lighting_msg.channel = channel
    lighting_msg.r = rgb[0]
    lighting_msg.g = rgb[1]
    lighting_msg.b = rgb[2]

    msg = d360frame.Digit360Message(lighting_control=lighting_msg)

    if dev.is_open:
        dev.send(msg)

    return msg

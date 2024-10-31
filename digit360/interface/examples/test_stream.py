# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import betterproto

from digit360.interface import proto as d360frame
from digit360.interface.digit360 import Digit360, TimeDelta
from digit360.interface.usb.usb import get_digit360_devices

# Example instantiation of Digit360Descriptor takes first device
descriptor = get_digit360_devices()[0] 

digit360 = Digit360(descriptor.data)

td_pressure_ap = TimeDelta("pressure_ap")

while digit360.is_open:
    data = digit360.read()

    frame_name, frame_type = betterproto.which_one_of(data, "type")

    # various streams can be access through d360frame
    if isinstance(frame_type, d360frame.PressureApData):
        td_pressure_ap(data.pressure_ap_data.ts)

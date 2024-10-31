# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


from digit360.interface.usb.usb import get_digit360_devices

# get sorted device list connected when multiple digit360 connected
connected_devices = get_digit360_devices()
print(connected_devices)
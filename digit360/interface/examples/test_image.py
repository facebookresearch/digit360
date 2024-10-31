# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
 
import cv2

from digit360.interface.usb.usb import get_digit360_devices

# Example instantiation of Digit360Descriptor takes first device
descriptor = get_digit360_devices()[0] 

dev = cv2.VideoCapture(descriptor.ics)

while True:
    ret, frame = dev.read()
    cv2.imshow('Digit 360', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

dev.release()
cv2.destroyAllWindows()
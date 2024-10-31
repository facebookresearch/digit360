# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import matplotlib.pyplot as plt
import numpy as np
import sounddevice as sd

from digit360.interface.usb.usb import get_digit360_devices
from digit360.interface.stream.audio import audio

device_idx = 0

descriptor = get_digit360_devices()[device_idx] 

device = descriptor.audio
sample_rate = 48000
duration = 10

data = sd.rec(
    int(duration * sample_rate), samplerate=sample_rate, channels=2, dtype="int16", device=device
)

print(f"Sampling for {duration} s...")
sd.wait()
print("Finished.")

time = np.arange(0, duration, 1 / sample_rate)

plt.plot(time, data)
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.title("Digit 360")
plt.legend(["stream 1", "stream 2"])
plt.show()


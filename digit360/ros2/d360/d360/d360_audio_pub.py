# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

#!/usr/bin/env python3
from typing import Any
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from d360_msgs.msg import AudioInfoD360, AudioDataD360
import sounddevice as sd
import numpy as np
import queue
import ctypes
from sounddevice import CallbackFlags
import matplotlib.pyplot as plt

class AudioStreamer(Node):
    def __init__(self) -> None:
        super().__init__('d360_audio_streamer')
        self.queue: queue.Queue = queue.Queue()

        self.declare_parameter('device_name', 'DIGIT360')
        self.device_name = self.get_parameter('device_name').value
        self.blocksize=512
        try:
            device = sd.query_devices(self.device_name)
        except ValueError:
            raise IOError(f"could not initialize D360 audio device_name:={self.device_name}")

        print('Found digit360 audio device', device)

        self.samplerate = device['default_samplerate']
        self.channels = device['max_input_channels']

        self.publisher_data= [ self.create_publisher(AudioDataD360, f'mic_{ch}', 10) for ch in range(self.channels)]

    def callback(self, indata: np.ndarray, frames: int, time: Any, status: CallbackFlags) -> None:
        assert self.blocksize == frames
        self.queue.put((time.inputBufferAdcTime, indata.copy()))

    def start_streaming(self) -> None:
        with sd.InputStream(samplerate=self.samplerate, blocksize=self.blocksize, device=self.device_name,
                            channels=self.channels,dtype='float32', callback=self.callback):
            
            while rclpy.ok():
                inputTime, audio_data = self.queue.get()
                msg = AudioDataD360()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.sample_rate = int(self.samplerate)
                for ch in range(self.channels):
                    msg.data = audio_data[:,ch].tolist()
                    self.publisher_data[ch].publish(msg)

def main():
    rclpy.init(args=None)
    sd._terminate()
    sd._initialize()
    audio_streamer = AudioStreamer()
    audio_streamer.start_streaming()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import proto as d360frame
import psutil
import serial
import numpy as np
from cobs import cobs
from dataclasses import dataclass

@dataclass
class Digit360Descriptor:
    serial: str
    data: str
    audio: str
    ics: str
    base_version: str
    ics_version: str

class TimeDelta:
    def __init__(self, name: str) -> None:
        self.name = name
        self.prev = 0.0
        self.cnt = 0
        self.td: dict = {}
        self.window = 10
        self.hz: list = []
        self.delta: list = []

    def step(self) -> None:
        self.cnt += 1

    def result(self) -> None:
        if self.cnt % self.window == 0:
            self.td = {
                "key": self.name,
                "ms": f"{np.average(self.delta):.2f}",
                "hz": f"{np.average(self.hz):.2f}",
            }
            print(self.td)
            self.hz = []
            self.delta = []
            self.cnt = 0
        self.step()

    def __call__(self, ts: float) -> None:
        delta = ts - self.prev
        self.delta.append(delta)
        if self.prev == ts:
            return
        self.prev = ts
        self.hz.append(1 / delta * 1e3)
        self.result()

class Digit360:
    def __init__(self, port: str, port_timeout: float = None) -> None:
        self.overruns = 0  # index, key errors
        self.tlc = 0  # total life count of packets
        self.cerr = 0  # decode errors

        self._device_buffer = bytearray()
        self._dev: serial.Serial = None
        self.port = port
        self.port_timeout = port_timeout

        self.connect()

        self.proc = psutil.Process()

    def connect(self) -> None:
        self._dev = serial.Serial(self.port, timeout=self.port_timeout)
        if not self._dev.is_open:
            raise IOError(f"Unable to access device at {self.port}!")
        self._dev.reset_input_buffer()

    def _read_device(self) -> bytearray:
        i = self._device_buffer.find(b"\x00")
        if i >= 0:
            r = self._device_buffer[: i + 1]
            self._device_buffer = self._device_buffer[i + 1 :]
            return r
        while True:
            i = max(1, min(2048, self._dev.in_waiting))
            data = self._dev.read(i)
            i = data.find(b"\x00")
            if i >= 0:
                r = self._device_buffer + data[: i + 1]
                self._device_buffer[0:] = data[i + 1 :]
                return r
            else:
                self._device_buffer.extend(data)

    def read(self) -> d360frame.Digit360Message:
        data = self._read_device()
        data = self.decode(data)
        return data

    def decode(self, data: bytes) -> d360frame.Digit360Message:
        cobs_data = data.split(b"\x00")[0]

        digit360_frame = d360frame.Digit360Message()

        try:
            pb_data = cobs.decode(cobs_data)
            digit360_frame = digit360_frame.parse(pb_data)
            self.tlc += 1
        except cobs.DecodeError:
            self._dev.reset_input_buffer()
            self.cerr += 1
            print("cobs error:", self.tlc)
        except IndexError:
            self._dev.reset_input_buffer()
            self.overruns += 1
            print("index error:", self.tlc)
            pass
        except KeyError:
            self._dev.reset_input_buffer()
            print("key error:", self.tlc)
            self.overruns += 1
            pass

        # periodic error logging and stats
        if self.tlc % 10000 == 0:
            p_err = ((self.cerr + self.overruns) / self.tlc) * 100.0
            print(
                f"tlc: {self.tlc}, errors: {self.overruns + self.cerr}, "
                f"%: {p_err:.2f}, cpu%: {self.proc.cpu_percent()}"
            )

        return digit360_frame

    def send(self, data: bytes) -> None:
        self._dev.write(bytes(data))
        self._dev.flush()

    @property
    def is_open(self) -> bool:
        return self._dev.is_open
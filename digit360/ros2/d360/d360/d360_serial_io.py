# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.


#!/usr/bin/env python3

from argparse import ArgumentParser
import betterproto
from cobs import cobs
import os
import rclpy.exceptions
import serial
import rclpy
import rclpy.qos
import rclpy.utilities
import rclpy.parameter
from typing import List, Tuple
import struct
from rclpy.node import Node
import numpy as np
from d360_msgs.msg import PressureD360, PressureAPD360, GasHTData, ImuRawD360, ImuEulerD360, ImuQuatD360
from builtin_interfaces.msg import Time
import time
from std_msgs.msg import Header
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
import warnings
from collections import deque
import yaml
from digit360.interface import proto as d360frame

class TimeDelta:
    def __init__(self, name: str) -> None:
        self.name:str  = name
        self.timestamps: deque= deque(maxlen=1000)
        self.count = 0
        self.report_freq = 1000
    
    def step(self, t_ms: float = None) -> None:
        if t_ms is None:
            t_ms = time.time()*1000
        self.timestamps.append(t_ms)
        if len(self.timestamps) == self.timestamps.maxlen:
            self.timestamps.popleft()

        self.count+=1

    def get_hz(self) -> float:
        delta = self.timestamps[-1] - self.timestamps[0]
        hz = 1000 * len(self.timestamps) / delta
        return hz
    
    def get_delta_stat(self) -> float:
        diff=np.diff(np.array(self.timestamps))
        mean = np.mean(diff)
        delta = np.std(diff)
        return mean, delta

    def report(self) -> str:
        return f"{self.name} Hz: {self.get_hz()}, mean/std: {self.get_delta_stat()} ms"
    
    def __call__(self, t_ms: float, silent:bool=False) -> str:
        "call this function periodically to have rate stats printed"
        self.step(t_ms)
        if self.count % self.report_freq == 0:
            msg=self.report()
            if not silent:
                print(msg)
            return msg
        return None

class Digit360():
    def __init__(self, port: str, port_timeout: float=1.0) -> None:
        self._dev = serial.Serial(port, timeout=port_timeout)
        self.overruns = 0
        self._device_buffer = bytearray()

        if not self._dev.is_open:
            raise IOError(f"Unable to access DIGIT360 data device at *{port}*!")
        self.readline = self.ReadLine(self._dev)

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
        if self._dev.in_waiting > 4000:
            warnings.warn(f"DIGIT360 data serial buffer piling up {self._dev.in_waiting} bytes")
        data = self._read_device()
        if len(data) == 0:
            raise RuntimeError(f"DIGIT360 data serial port *{self._dev}* read timeout, wrong port?")
        data = self.decode(data)
        return data
    
    def decode(self, data: bytes) -> d360frame.Digit360Message:
        cobs_data = data.split(b"\x00")[0]
        digit360_frame = d360frame.Digit360Message()

        try:
            pb_data = cobs.decode(cobs_data)
            digit360_frame = digit360_frame.parse(pb_data)
        except cobs.DecodeError:
            self._dev.reset_input_buffer()
            warnings.warn("DIGIT360 Data::Decode::DecodeError cobs cannot be decoded")
        except IndexError:
            self.overruns += 1
            warnings.warn(f"DIGIT360 Data::Decode::IndexError, overrun {self.overruns}")
            self._dev.reset_input_buffer()
        except KeyError:
            self.overruns += 1
            warnings.warn(f"DIGIT360 Data::Decode::KeyError, overrun {self.overruns}")
            self._dev.reset_input_buffer()
        return digit360_frame
    
    def _send(self, data: bytes) -> None:
        self._dev.write(bytes(data))
        self._dev.flush()

    @property
    def is_open(self) -> bool:
        return self._dev.is_open

class D360Node(Node):
    def __init__(self) -> None:
        super().__init__('d360_node')

        self.n_led = 8

        # if there is a hydra parameter
        # Declare parameters for lighting control
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyACM0'),
                ("reliable_publisher", False),
                ("config_file", "default.yaml"),
                ("hand_config","default.yaml")
            ]
        )

        # Get the config file path from parameters
        config_file_path = self.get_parameter('config_file').get_parameter_value().string_value
        hand_config_path = self.get_parameter('hand_config').get_parameter_value().string_value
        
        print(f'DIGITG360 Node config file: {config_file_path}')
        print(f'DIGITG360 Node hand file: {hand_config_path}')

        if (lighting_params := self.load_lighting_params_from_yaml(hand_config_path)) is not None:
            self.lighting_params_from_yaml = lighting_params
            print(f"light loaded from hand config{self.lighting_params_from_yaml}")
        else:
            self.lighting_params_from_yaml = self.load_lighting_params_from_yaml(config_file_path)
            print(f"loaded from individual cfg path")

        # If YAML parameters exist and are complete, use them. Otherwise, use default values.
        if self.lighting_params_from_yaml:
            led_params = self.lighting_params_from_yaml
        else:
            led_params = self.get_default_led_params()

        self.declare_parameters(
            namespace="",
            parameters=[*led_params]
            )

        # lightcontrol dynamic param callback
        self.add_on_set_parameters_callback(self.check_param_updates)

        #  publishers for pressure, IMU, and gas data
        self.publisher_pressure = self.create_publisher(
            PressureD360, "pressure_topic", 1
        )
        self.publisher_imu_raw = self.create_publisher(ImuRawD360, "imu_raw_topic", 1)
        self.publisher_imu_euler = self.create_publisher(
            ImuEulerD360, "imu_euler_topic", 1
        )
        self.publisher_imu_quat = self.create_publisher(
            ImuQuatD360, "imu_quat_topic", 1
        )
        self.publisher_gas_hum_temp = self.create_publisher(GasHTData, "ght_topic", 1)

        if self.get_parameter("reliable_publisher").value:
            qos_reliable = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            )
            self.publisher_pressure_ap = self.create_publisher(
                PressureAPD360, "pressure_ap_topic", qos_profile=qos_reliable
            )
        else:
            self.publisher_pressure_ap = self.create_publisher(
                PressureAPD360, "pressure_ap_topic", 1
            )

        self.publisher_pressure_ap_seq = 0

        self.digit360 = Digit360(self.get_parameter("port").value)

        self.parameter_dirty = True
        self.timer_pressure_ap = TimeDelta("pressure_ap")


    def load_lighting_params_from_yaml(self, config_file: str) -> List[Tuple[str, int]]:
        if os.path.exists(config_file):
            with open(config_file, 'r') as file:
                yaml_config = yaml.safe_load(file)

            # Navigate to the 'lighting' section inside 'params'
            if 'params' in yaml_config and 'lighting' in yaml_config['params']:
                lighting_config = yaml_config['params']['lighting']
                
                # Define all required lighting parameter keys
                required_lighting_params = [
                    'lighting_alpha',
                    'lighting_1_r', 'lighting_1_g', 'lighting_1_b',
                    'lighting_2_r', 'lighting_2_g', 'lighting_2_b',
                    'lighting_3_r', 'lighting_3_g', 'lighting_3_b',
                    'lighting_4_r', 'lighting_4_g', 'lighting_4_b',
                    'lighting_5_r', 'lighting_5_g', 'lighting_5_b',
                    'lighting_6_r', 'lighting_6_g', 'lighting_6_b',
                    'lighting_7_r', 'lighting_7_g', 'lighting_7_b',
                    'lighting_8_r', 'lighting_8_g', 'lighting_8_b'
                ]
                # Check if all required lighting parameters exist in the YAML config
                missing_params = [param for param in required_lighting_params if param not in lighting_config]

                if missing_params:
                    self.get_logger().warn(f"Missing lighting parameters: {missing_params}")
                    return None  # Indicate that parameters are incomplete
                else:
                    # Return parameters from YAML
                    return [(key, lighting_config[key]) for key in required_lighting_params]
            else:
                self.get_logger().warn(f"Lighting parameters not found in the config file.{config_file}")
                return None

        else:
            self.get_logger().warn(f"Config file {config_file} does not exist.")
            return None


    def get_default_led_params(self):
        """
        Get default lighting parameters if no YAML file is provided.
        """
        print(f" loading default light params - NO LIGHTING PARAMS FROM config file")
        led_params = []
        led_params.append((f'lighting_alpha', 255))
        for ch in range(1, self.n_led + 1):
            led_params.append((f'lighting_{ch}_r', int(ch % 3 == 0) * 15))
            led_params.append((f'lighting_{ch}_g', int(ch % 3 == 1) * 15))
            led_params.append((f'lighting_{ch}_b', int(ch % 3 == 2) * 15))
        return led_params

    def check_param_updates(
        self, parameters: List[rclpy.parameter.Parameter]
    ) -> SetParametersResult:
        result = SetParametersResult(successful=True)
        for p in parameters:
            if p.name == "port":
                result.successful = False
                result.reason = (
                    "port cannot be dynamically changed, please restart the node"
                )
                return result
            if p.name.startswith("lighting_"):
                if p.type_ != p.Type.INTEGER:
                    result.successful = False
                    result.reason = "lighting value must be an integer"
                    return result

        self.parameter_dirty = True
        return result

    def update_lighting_parameters(self) -> None:
        """
        Update lighting parameters and write them to the Digit360 device.
        """
        new_lighting_alpha = self.get_parameter(f'lighting_alpha').value
        for ch in range(1, self.n_led+1):
            new_lighting_r = self.get_parameter(f'lighting_{ch}_r').value
            new_lighting_g = self.get_parameter(f'lighting_{ch}_g').value
            new_lighting_b = self.get_parameter(f'lighting_{ch}_b').value
            # Write light values to Digit360 using the function
            self.write_lighting_to_digit360(new_lighting_alpha, ch, new_lighting_r, new_lighting_g, new_lighting_b)

    def write_lighting_to_digit360(self, alpha: int, channel:int, r:int, g:int, b:int) -> None:
        """
        Write lighting values to Digit360 device.
        """
        # Form a protobuf message for lighting
        lighting_msg = d360frame.LightingControl()
        lighting_msg.alpha = alpha
        lighting_msg.channel = d360frame.LightingChannel(channel)
        lighting_msg.r = r
        lighting_msg.g = g
        lighting_msg.b = b

        message = d360frame.Digit360Message(lighting_control=lighting_msg)
        # Write the lighting data to the Digit360 device
        self.get_logger().info(f"send {message}")
        self.digit360._send(message)

    def process_pressure_data(self, data:d360frame.PressureData) -> None:
        pressure_msg = PressureD360()
        time_pressure_msg = Time()

        # Assign data to the header - ros time
        pressure_msg.header.stamp = self.get_clock().now().to_msg()
        pressure_msg.header.frame_id = "D360"

        # Create and publish pressure message
        pressure_msg.ts = data.pressure_data.ts
        pressure_msg.pressure = data.pressure_data.pressure
        pressure_msg.temperature = data.pressure_data.temperature

        self.publisher_pressure.publish(pressure_msg)

    def process_pressure_ap_data(self, data:d360frame.PressureApData) -> None:
        channel_a = data.pressure_ap_data.channel_a
        channel_b = data.pressure_ap_data.channel_b

        pres_ap_msg = PressureAPD360()
        pres_ap_msg.seq = self.publisher_pressure_ap_seq
        self.publisher_pressure_ap_seq += 1
        
        # Assign data to the header - ros time
        pres_ap_msg.header.stamp = self.get_clock().now().to_msg()
        pres_ap_msg.header.frame_id = "D360"

        # Device time
        pres_ap_msg.ts = data.pressure_ap_data.ts
        
        pres_ap_msg.ch_a.extend(np.frombuffer(channel_a, dtype=np.uint16))
        pres_ap_msg.ch_b.extend(np.frombuffer(channel_b, dtype=np.uint16))
        self.publisher_pressure_ap.publish(pres_ap_msg)

    def process_imu_raw_data(self, data:d360frame.ImuData) -> None:
        imu_msg = ImuRawD360()

        # Assign data to the header - ros time
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "D360"
        
        # Device time
        imu_msg.ts = data.imu_data.ts
        
        # Sensor time
        imu_msg.sensor_ts = data.imu_data.raw.ts_ght
        
        imu_msg.type = data.imu_data.raw.sensor
        imu_msg.x = data.imu_data.raw.x
        imu_msg.y = data.imu_data.raw.y
        imu_msg.z = data.imu_data.raw.z

        self.publisher_imu_raw.publish(imu_msg)

    def process_imu_euler_data(self, data:d360frame.ImuData) -> None:
        imu_msg = ImuEulerD360()

        # Assign data to the header - ros time
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "D360"

        # Device time
        imu_msg.ts = data.imu_data.ts
        
        # Sensor time
        imu_msg.sensor_ts = data.imu_data.euler.ts_ght
        imu_msg.heading = data.imu_data.euler.heading
        imu_msg.pitch = data.imu_data.euler.pitch
        imu_msg.roll = data.imu_data.euler.roll

        self.publisher_imu_euler.publish(imu_msg)

    def process_imu_quat_data(self, data:d360frame.ImuData) -> None:
        imu_msg = ImuQuatD360()

        # Assign data to the header - ros time
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "D360"

        # Device time
        imu_msg.ts = data.imu_data.ts

        # Sensor time
        imu_msg.sensor_ts = data.imu_data.quat.ts_ght
        imu_msg.quat.x = data.imu_data.quat.x
        imu_msg.quat.y = data.imu_data.quat.y
        imu_msg.quat.z = data.imu_data.quat.z
        imu_msg.quat.w = data.imu_data.quat.w
        imu_msg.accuracy = data.imu_data.quat.accuracy

        self.publisher_imu_quat.publish(imu_msg)

    def process_gasht_data(self, data:d360frame.GasHtData) -> None:
        ght_msg = GasHTData()
        
        # Assign data to the header - ros time
        ght_msg.header.stamp = self.get_clock().now().to_msg()
        ght_msg.header.frame_id = "D360"

        # Device time
        ght_msg.ts = data.gasht_data.ts

        # Sensor time
        ght_msg.sensor_ts = data.gasht_data.ts_ght
        
        ght_msg.temperature = data.gasht_data.temperature
        ght_msg.pressure = data.gasht_data.pressure
        ght_msg.humidity = data.gasht_data.humidity
        ght_msg.gas = data.gasht_data.gas
        ght_msg.gas_index = int(data.gasht_data.gas_index)

        self.publisher_gas_hum_temp.publish(ght_msg)

    def process_data(self, data:d360frame.Digit360Message) -> None:
        frame_name, frame_type = betterproto.which_one_of(data, "type")
        if isinstance(frame_type, d360frame.PressureData):
            self.process_pressure_data(data)
        elif isinstance(frame_type, d360frame.PressureApData):
            msg = self.timer_pressure_ap(data.pressure_ap_data.ts, silent=True)
            if msg:
                self.get_logger().info(msg)
            self.process_pressure_ap_data(data)
        elif isinstance(frame_type, d360frame.ImuData):
            sensor_name, sensor_type = betterproto.which_one_of(data.imu_data, "imu_type")
            if isinstance(sensor_type, d360frame.RawImuData):
                self.process_imu_raw_data(data)
            elif isinstance(sensor_type, d360frame.EulerData):
                self.process_imu_euler_data(data)
            elif isinstance(sensor_type, d360frame.QuatData):
                self.process_imu_quat_data(data)
            else:
                self.get_logger().warn(f"not implemented imu sensor type {sensor_type}")
        elif isinstance(frame_type, d360frame.GasHtData):
            self.process_gasht_data(data)

    def fake_run(self) -> None:
        while True:
            data = d360frame.Digit360Message(pressure_ap_data=d360frame.PressureApData(channel_a=b'1'*1000, channel_b=b'2'*1000))
            self.process_data(data)
            time.sleep(0.001)

    def run(self) -> None:
        try:
            while rclpy.ok():  # Continuously loop until ROS is shutdown
                try:
                    with warnings.catch_warnings(record=True) as warnings_list:
                        data = self.digit360.read()
                    for w in warnings_list:
                        self.get_logger().warn(f'{os.path.basename(w.filename)}:{w.lineno} {w.message}', throttle_duration_sec=1, skip_first=1)

                    self.process_data(data)
                except (struct.error, ValueError, AssertionError) as e:
                    # decoding error in first second
                    self.get_logger().warn(str(e), throttle_duration_sec=1, skip_first=1)

                if self.parameter_dirty:
                    self.update_lighting_parameters()
                    self.parameter_dirty=False

        except (KeyboardInterrupt, rclpy.exceptions.InvalidHandle) as e:
            pass

def main(rawargs:list=None) -> None:
    rclpy.init(args=rawargs)
    stripped_args = rclpy.utilities.remove_ros_args(rawargs)
    parser = ArgumentParser()
    parser.add_argument("--fake", action="store_true", help="Run in test fake data mode")
    args = parser.parse_args(args=stripped_args[1:])
    node = D360Node()
    
    e = MultiThreadedExecutor()

    e.add_node(node)
    if args.fake:
        future=e.create_task(node.fake_run)
    else:
        future=e.create_task(node.run)
    try:
        e.spin_until_future_complete(future)
        future.result()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

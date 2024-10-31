# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import os
import torch
import random
import numpy as np
from glob import glob
from torch.utils.data import Dataset
from d360.utils import normalize_data


class D360(Dataset):
    def __init__(
        self, data_root, split, modality, t_downsample=4, traj_length=10
    ):
        self.data_root = data_root
        self.split = split
        self.modality = modality
        self.traj_length = traj_length
        self.t_downsample = t_downsample
        # record valid starting index according to trajectory length
        # avoid the jump between temporal sequences
        self.traj_info = np.zeros(0, dtype=np.uint32)
        self.traj_rs_rgb = np.zeros((0, 160, 160, 3), dtype=np.float32)
        self.traj_labels = np.zeros(0, dtype=np.float32)
        self.traj_materials = np.zeros(0, dtype=np.float32)
        if modality.independent:
            self.traj_d360 = np.zeros((0, 160, 160, 3), dtype=np.float32)
            self.traj_mic = np.zeros((0, 512), dtype=np.float32)
            self.traj_pressure = np.zeros((0, 1 * t_downsample), dtype=np.float32)
            self.traj_imus = np.zeros((0, 3), dtype=np.float32)
            self.traj_specgram = np.zeros((0, 64, 1), dtype=np.float32)
        else:
            self.traj_d360 = np.zeros((0, 160, 160, 12), dtype=np.float32)
            self.traj_mic = np.zeros((0, 2048), dtype=np.float32)
            self.traj_pressure = np.zeros((0, 4 * t_downsample), dtype=np.float32)
            self.traj_imus = np.zeros((0, 12), dtype=np.float32)
            self.traj_specgram = np.zeros((0, 64, 4), dtype=np.float32)
        label_names = os.path.join(data_root, split, '*_labels.npy')
        label_list = sorted(glob(label_names))
        for label_name in label_list:
            # parse images
            # _rs = self._load_data(label_name.replace('_labels', '_images'))
            # self.traj_rs_rgb = np.concatenate([self.traj_rs_rgb, _rs], axis=0)
            # parse digit images
            _d360 = self._load_data(label_name.replace('_labels', '_d360'))
            _d360 = _d360.transpose(0, 2, 3, 4, 1)
            _d360 = _d360.reshape(_d360.shape[:3] + (12,))
            num_sliding_window = _d360.shape[0] - traj_length + 1
            if num_sliding_window < 0:
                continue
            if modality.independent:
                for i in range(4):
                    _d360_i = _d360[..., i * 3:i * 3 + 3]
                    sw_idx = self.traj_d360.shape[0] + np.arange(num_sliding_window)
                    self.traj_info = np.hstack([self.traj_info, sw_idx])
                    self.traj_d360 = np.concatenate([self.traj_d360, _d360_i], axis=0)
            else:
                sw_idx = self.traj_d360.shape[0] + np.arange(num_sliding_window)
                self.traj_info = np.hstack([self.traj_info, sw_idx])
                self.traj_d360 = np.concatenate([self.traj_d360, _d360], axis=0)
            # parse digit audio
            _mic = self._load_data(label_name.replace('_labels', '_mics'))
            if modality.independent:
                for i in range(4):
                    _mic_i = _mic[:, i]
                    self.traj_mic = np.concatenate([self.traj_mic, _mic_i], axis=0)
            else:
                _mic = _mic.reshape((_mic.shape[0], -1))
                self.traj_mic = np.concatenate([self.traj_mic, _mic], axis=0)
            # parse digit specgram
            _specgram = self._load_data(
                label_name.replace('_labels', '_specgram'), specgram=True
            )
            pad_length = _mic.shape[0] * 4 - len(_specgram)
            pad_array = _specgram[[-1]].repeat(pad_length, axis=0)
            _specgram = np.concatenate([_specgram, pad_array], axis=0)
            if modality.independent:
                for i in range(4):
                    _specgram_i = _specgram[..., [i]]
                    self.traj_specgram = np.concatenate([self.traj_specgram, _specgram_i], axis=0)
            else:
                self.traj_specgram = np.concatenate([self.traj_specgram, _specgram], axis=0)
            # parse digit pressure (Tx4x1)
            # instead of directly downsample, we concat at the last dim
            _pressure = np.load(label_name.replace('_labels', '_pressure'))
            pad_length = _mic.shape[0] * t_downsample - len(_pressure)
            pad_array = _pressure[[-1]].repeat(pad_length, axis=0)
            _pressure = np.concatenate([_pressure, pad_array], axis=0)
            if modality.independent:
                for i in range(4):
                    _pressure_i = _pressure[:, i]
                    _pressure_i = _pressure_i.reshape(
                        (_pressure_i.shape[0] // t_downsample, t_downsample)
                    )
                    self.traj_pressure = np.concatenate([self.traj_pressure, _pressure_i], axis=0)
            else:
                _pressure = _pressure.reshape(
                    (_pressure.shape[0] // t_downsample, t_downsample * 4)
                )
                self.traj_pressure = np.concatenate([self.traj_pressure, _pressure], axis=0)
            # parse digit imu
            _imus = self._load_data(label_name.replace('_labels', '_imus'))
            if modality.independent:
                for i in range(4):
                    _imus_i = _imus[:, i]
                    self.traj_imus = np.concatenate([self.traj_imus, _imus_i], axis=0)
            else:
                _imus = _imus.reshape((_imus.shape[0], -1))
                self.traj_imus = np.concatenate([self.traj_imus, _imus], axis=0)
            # parse material
            material_name = label_name.replace('_labels', '_material')
            material = np.repeat(np.load(material_name), num_sliding_window)
            label = np.repeat(np.load(label_name), num_sliding_window)
            if modality.independent:
                for i in range(4):
                    self.traj_materials = np.hstack([self.traj_materials, material])
                    self.traj_labels = np.hstack([self.traj_labels, label])
            else:
                self.traj_materials = np.hstack([self.traj_materials, material])
                self.traj_labels = np.hstack([self.traj_labels, label])

        # heuristic data normalization, might be suboptimal
        # empirically better than running_mean_std
        self.traj_rs_rgb /= 255
        self.traj_d360 /= 255
        self.traj_mic = np.clip(self.traj_mic, -1e-2, 1e-2)
        self.traj_mic *= 100
        self.traj_imus = np.clip(self.traj_imus, -1e5, 1e5)
        self.traj_imus /= 1000

    def _load_data(self, file_name, specgram=False):
        signal = np.load(file_name)
        if specgram:
            signal = signal.transpose((2, 1, 0))
        if not specgram:
            signal = signal[::self.t_downsample]
        return signal

    def __len__(self):
        return self.traj_info.shape[0]

    def __getitem__(self, idx):
        traj_id = self.traj_info[idx]
        data_dict = {}
        # images = self.traj_rs_rgb[traj_id:traj_id+self.traj_length]
        # images = torch.from_numpy(images.astype(np.float32))
        if self.modality.rgb:
            d360 = self.traj_d360[traj_id:traj_id + self.traj_length]
            # if self.split == 'train':
            #     if random.random() < 0.5:
            #         d360 = d360[:, ::-1]
            #     if random.random() < 0.5:
            #         d360 = d360[:, :, ::-1]
            d360 = torch.from_numpy(d360.astype(np.float32))
            data_dict['rgb'] = d360
        if self.modality.imu:
            imus = self.traj_imus[traj_id:traj_id + self.traj_length]
            imus = torch.from_numpy(imus.astype(np.float32))
            data_dict['imu'] = imus
        if self.modality.mic:
            mic = self.traj_mic[traj_id:traj_id + self.traj_length]
            mic = torch.from_numpy(mic.astype(np.float32))
            data_dict['mic'] = mic
        if self.modality.specgram:
            specgram = self.traj_specgram[
                traj_id * self.t_downsample:
                traj_id * self.t_downsample + self.traj_length * self.t_downsample
            ]
            specgram = torch.from_numpy(specgram.astype(np.float32))
            data_dict['specgram'] = specgram
        if self.modality.pressure:
            pressure = self.traj_pressure[traj_id:traj_id + self.traj_length]
            pressure = torch.from_numpy(pressure.astype(np.float32))
            data_dict['pressure'] = pressure
        material = torch.from_numpy(self.traj_materials[[idx]].astype(np.int32))
        labels = torch.from_numpy(self.traj_labels[[idx]].astype(np.int32))
        return data_dict, material, labels

# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import torch
import torchvision
import torch.nn as nn
from dataclasses import dataclass


class BasicBlock(nn.Module):
    expansion = 1

    def __init__(self, in_planes, planes, stride=1):
        super(BasicBlock, self).__init__()
        self.conv1 = nn.Conv2d(
            in_planes, planes, kernel_size=3, stride=stride, padding=1, bias=False)
        self.bn1 = nn.GroupNorm(8, planes)
        self.conv2 = nn.Conv2d(planes, planes, kernel_size=3,
                               stride=1, padding=1, bias=False)
        self.bn2 = nn.GroupNorm(8, planes)

        self.shortcut = nn.Sequential()
        if stride != 1 or in_planes != self.expansion*planes:
            self.shortcut = nn.Sequential(
                nn.Conv2d(in_planes, self.expansion*planes,
                          kernel_size=1, stride=stride, bias=False),
                nn.GroupNorm(8, self.expansion*planes)
            )

    def forward(self, x):
        out = torch.relu(self.bn1(self.conv1(x)))
        out = self.bn2(self.conv2(out))
        out += self.shortcut(x)
        out = torch.relu(out)
        return out


class ResNet(nn.Module):
    def __init__(self, block, num_blocks):
        super(ResNet, self).__init__()
        self.in_planes = 32

        self.conv1 = nn.Conv2d(3, 32, kernel_size=3,
                               stride=2, padding=3, bias=False)
        self.bn1 = nn.GroupNorm(8, 32)
        self.layer1 = self._make_layer(block, 32, num_blocks[0], stride=2)
        self.layer2 = self._make_layer(block, 64, num_blocks[1], stride=2)
        self.layer3 = self._make_layer(block, 128, num_blocks[2], stride=2)
        self.layer4 = self._make_layer(block, 256, num_blocks[3], stride=2)
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))

    def _make_layer(self, block, planes, num_blocks, stride):
        strides = [stride] + [1]*(num_blocks-1)
        layers = []
        for stride in strides:
            layers.append(block(self.in_planes, planes, stride))
            self.in_planes = planes * block.expansion
        return nn.Sequential(*layers)

    def forward(self, x):
        out = torch.relu(self.bn1(self.conv1(x)))
        out = self.layer1(out)
        out = self.layer2(out)
        out = self.layer3(out)
        out = self.layer4(out)
        out = self.avgpool(out)
        out = out.view(out.size(0), -1)
        return out


@dataclass
class ModelInput:
    rgb: torch.Tensor = None
    imu: torch.Tensor = None
    mic: torch.Tensor = None
    specgram: torch.Tensor = None
    pressure: torch.Tensor = None


class Net(nn.Module):
    def __init__(self, config):
        super(Net, self).__init__()
        self.config = config
        self.feat_dim = 0
        divider = 4 if config.independent else 1
        if config.rgb:
            # self.image_encoder = torchvision.models.resnet18()
            self.image_encoder = ResNet(BasicBlock, [2, 2, 2, 2])
            self.image_encoder.conv1 = nn.Conv2d(
                12 // divider * config.num_finger // 4, 32, kernel_size=7, stride=2, padding=3, bias=False
            )
            self.feat_dim += 256

        if config.imu:
            self.imu_encoder = nn.Sequential(
                nn.Linear(12 // divider * config.num_finger // 4, 1024),
                nn.ReLU(),
                nn.Linear(1024, 1024),
                nn.ReLU(),
                nn.Linear(1024, 1024),
                nn.ReLU(),
                nn.Linear(1024, 256),
            )
            self.feat_dim += 256

        if config.mic:
            self.mic_encoder = nn.Sequential(
                nn.Linear(2048 // divider, 1024),
                nn.ReLU(),
                nn.Linear(1024, 1024),
                nn.ReLU(),
                nn.Linear(1024, 1024),
                nn.ReLU(),
                nn.Linear(1024, 256),
            )
            self.feat_dim += 256

        if config.specgram:
            self.specgram_encoder = ResNet(BasicBlock, [2, 2, 2, 2])
            self.specgram_encoder.conv1 = nn.Conv2d(
                4 // divider * config.num_finger // 4, 32, kernel_size=7, stride=2, padding=3, bias=False
            )
            self.specgram_encoder.fc = nn.Identity()
            self.feat_dim += 256

        if config.pressure:
            self.pressure_encoder = nn.Sequential(
                nn.Linear(16 // divider * config.num_finger // 4, 512),
                nn.ReLU(),
                nn.Linear(512, 512),
                nn.ReLU(),
                nn.Linear(512, 512),
                nn.ReLU(),
                nn.Linear(512, 128),
            )
            self.feat_dim += 128

        self.act_mlp = nn.Sequential(
            nn.Linear(self.feat_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 3),
        )

        self.mat_mlp = nn.Sequential(
            nn.Linear(self.feat_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 3),
        )

        num_params = sum(p.numel() for p in self.parameters())
        print(f'number of parameters: {num_params}')

    def forward(self, x: ModelInput):
        feat = []
        # (N, T, H, W, 3) -> (N, T, 3, H, W)
        if self.config.rgb:
            rgb = x.rgb
            batch, t = rgb.shape[:2]
            rgb = rgb.permute(0, 1, 4, 2, 3)
            rgb = rgb.reshape((-1,) + rgb.shape[2:])
            rgb = self.image_encoder(rgb)
            rgb = rgb.reshape(batch, t, -1)
            rgb = rgb.mean(dim=1)
            feat.append(rgb)

        if self.config.imu:
            imu = x.imu
            imu = self.imu_encoder(imu)
            imu = imu.mean(dim=1)
            feat.append(imu)

        if self.config.mic:
            mic = x.mic
            mic = self.mic_encoder(mic)
            mic = mic.mean(dim=1)
            feat.append(mic)

        if self.config.specgram:
            specgram = x.specgram
            specgram = specgram.permute(0, 3, 1, 2)
            specgram = self.specgram_encoder(specgram)
            feat.append(specgram)

        if self.config.pressure:
            pressure = x.pressure
            pressure = self.pressure_encoder(pressure)
            pressure = pressure.mean(dim=1)
            feat.append(pressure)

        x = torch.cat(feat, dim=1)
        return self.act_mlp(x), self.mat_mlp(x)

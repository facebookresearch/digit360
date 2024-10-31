# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import math
import time
import torch
import numpy as np
from torch import nn
from d360.model import ModelInput
from tensorboardX import SummaryWriter


class Trainer(object):
    def __init__(
        self, config, device, train_loader, val_loader,
        model, optim, output_dir
    ):
        # misc
        self.config = config
        self.device = device
        self.output_dir = output_dir
        # data loading
        self.train_loader, self.val_loader = train_loader, val_loader
        # nn optimization
        self.model = model
        self.optim = optim
        self.loss = nn.CrossEntropyLoss()
        self.max_epoch = 200
        self.max_iter = self.max_epoch * len(self.train_loader)
        self.base_lr = self.optim.defaults['lr']
        # tensorboard
        self.epoch_idx = 0
        self.iter_idx = 0
        self.writer = SummaryWriter(self.output_dir)
        self.num_finger = config.num_finger
        self.finger_idx = [int(i) for i in list(config.finger_idx)]
        assert len(self.finger_idx) == self.num_finger

    def train(self):
        self.model.train()
        print('\r', end='')
        t = time.time()
        for _ in range(self.max_epoch):
            self.train_epoch()
            self.model.eval()
            self.val()
            self.model.train()
            self.epoch_idx += 1
            self._adjust_learning_rate()
            epoch_time = (time.time() - t) / (_ + 1)
            print(f'Epoch {_} | Time: {epoch_time:.1f}s')

    def train_epoch(self):
        correct_act, correct_mat, total = 0, 0, 0
        for batch_idx, (data_dict, materials, labels) in enumerate(self.train_loader):
            labels = labels.to(self.device)
            materials = materials.to(self.device)
            model_input = self._prepare_inputs(data_dict)
            act, mat = self.model(model_input)
            # visualize image sequence
            # vis_image = images[0]
            # from matplotlib import pyplot as plt
            # for i in range(9):
            #     plt.subplot(3, 3, i + 1)
            #     plt.imshow(vis_image[i].cpu().numpy() / 255)
            # plt.show()
            self.optim.zero_grad()
            loss = (self.loss(act, labels[:, 0].long()) +
                    self.loss(mat, materials[:, 0].long()))
            loss.backward()
            self.optim.step()
            # calculate accuracy
            _, predict_act = act.max(1)
            _, predict_mat = mat.max(1)
            total += labels.shape[0]
            correct_act += predict_act.eq(labels[:, 0]).sum().item()
            correct_mat += predict_mat.eq(materials[:, 0]).sum().item()
            self.writer.add_scalar('train_loss', loss.item(), self.iter_idx)
            self.iter_idx += 1
        self.writer.add_scalar('train_acc_act', correct_act / total, self.epoch_idx)
        self.writer.add_scalar('train_acc_mat', correct_mat / total, self.epoch_idx)
        print('Action Train Acc: ', correct_act / total)
        print('Material Train Acc: ', correct_mat / total)

    @torch.no_grad()
    def val(self):
        correct_act, correct_mat, total = 0, 0, 0
        confusion_mat_info = np.zeros((0, 4))
        for batch_idx, (data_dict, materials, labels) in enumerate(self.val_loader):
            labels = labels.to(self.device)
            materials = materials.to(self.device)
            model_input = self._prepare_inputs(data_dict)
            act, mat = self.model(model_input)
            # calculate accuracy
            _, predict_act = act.max(1)
            _, predict_mat = mat.max(1)
            # predict_act[:] = torch.randint(0, 3, predict_act.shape)
            # predict_mat[:] = torch.randint(0, 3, predict_mat.shape)
            total += labels.shape[0]
            correct_act += predict_act.eq(labels[:, 0]).sum().item()
            correct_mat += predict_mat.eq(materials[:, 0]).sum().item()
            _cmat_info = np.concatenate([
                predict_act.cpu().numpy()[:, None], labels[:, [0]].cpu().numpy(),
                predict_mat.cpu().numpy()[:, None], materials[:, [0]].cpu().numpy(),
            ], axis=1)
            confusion_mat_info = np.concatenate([
                confusion_mat_info, _cmat_info
            ], axis=0)
        torch.save(self.model.state_dict(), f'{self.output_dir}/last.ckpt')
        np.save(f'{self.output_dir}/cmap_info.npy', confusion_mat_info)
        self.writer.add_scalar('test_acc_act', correct_act / total, self.epoch_idx)
        self.writer.add_scalar('test_acc_mat', correct_mat / total, self.epoch_idx)
        print('Action Test Acc: ', correct_act / total)
        print('Material Test Acc: ', correct_mat / total)
        # exit()

    def _prepare_inputs(self, data_dict):
        model_input = ModelInput()
        if self.config.rgb:
            idx = np.zeros(0, dtype=int)
            for i in self.finger_idx:
                idx = np.hstack([idx, np.arange(i * 3, i * 3 + 3)])
            model_input.rgb = data_dict['rgb'].to(self.device)
            model_input.rgb = model_input.rgb[..., idx]
        if self.config.imu:
            idx = np.zeros(0, dtype=int)
            for i in self.finger_idx:
                idx = np.hstack([idx, np.arange(i * 3, i * 3 + 3)])
            model_input.imu = data_dict['imu'].to(self.device)
            model_input.imu = model_input.imu[..., idx]
        if self.config.specgram:
            idx = np.zeros(0, dtype=int)
            for i in self.finger_idx:
                idx = np.hstack([idx, np.arange(i, i + 1)])
            model_input.specgram = data_dict['specgram'].to(self.device)
            model_input.specgram = model_input.specgram[..., idx]
        if self.config.mic:
            assert self.num_finger == 4
            model_input.mic = data_dict['mic'].to(self.device)
        if self.config.pressure:
            idx = np.zeros(0, dtype=int)
            for i in self.finger_idx:
                idx = np.hstack([idx, np.arange(i * 4, i * 4 + 4)])
            model_input.pressure = data_dict['pressure'].to(self.device)
            model_input.pressure = model_input.pressure[..., idx]
        return model_input

    def _adjust_learning_rate(self):
        lr = self.base_lr * 0.5 * (
            1. + math.cos(math.pi * self.epoch_idx / self.max_epoch)
        )
        for param_group in self.optim.param_groups:
            param_group['lr'] = lr
        return lr

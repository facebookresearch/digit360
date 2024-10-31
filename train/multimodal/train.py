# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import os
import tyro
import torch
import random
import numpy as np

from dataclasses import dataclass
from d360.model import Net
from d360.datasets import D360
from d360.trainer import Trainer


@dataclass(frozen=True)
class ModalityConfig:
    rgb: bool = False
    imu: bool = False
    mic: bool = False
    specgram: bool = False
    pressure: bool = False
    independent: bool = False
    seed: int = 0
    num_finger: int = 4
    finger_idx: str = '0123'
    output: str = 'debug'
    data_path: str = 'data/cls'


def get_data_loader(data_root, modality, batch_size=64):
    # dataset
    # dataloader
    train_set = D360(data_root=data_root, modality=modality, split='train')
    test_set = D360(data_root=data_root, modality=modality, split='val')
    kwargs = {'pin_memory': True, 'num_workers': 4}
    train_loader = torch.utils.data.DataLoader(
        train_set, batch_size=batch_size, shuffle=True, drop_last=True, **kwargs,
    )
    test_loader = torch.utils.data.DataLoader(
        test_set, batch_size=batch_size, shuffle=False, **kwargs,
    )
    print(f'size: train {len(train_loader)} / test {len(test_loader)}')
    return train_loader, test_loader


def main():
    config = tyro.cli(ModalityConfig)
    seed = config.seed
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.backends.cudnn.deterministic = True
        torch.cuda.manual_seed(seed)
    else:
        assert NotImplementedError

    train_loader, test_loader = get_data_loader(config.data_path, config)
    model = Net(config)
    model.to(torch.device('cuda'))
    optim = torch.optim.Adam(
        model.parameters(),
        lr=3e-4, weight_decay=0,
    )
    output_dir = os.path.join('outputs', config.output)
    os.makedirs(output_dir, exist_ok=True)

    trainer = Trainer(
        config, torch.device('cuda'), train_loader, test_loader,
        model, optim, output_dir
    )
    trainer.train()


if __name__ == '__main__':
    main()

# Copyright (c) Meta Platforms, Inc. and affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

def normalize_data(data):
    # normalize to [-1, 1]
    data_min = data.min(axis=0)
    data_max = data.max(axis=0)
    _data = (data - data_min) / (data_max - data_min + 1e-7)
    _data = _data * 2 - 1
    return _data

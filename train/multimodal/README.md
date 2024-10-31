# Multimodal Classification for Digit360

We provide instructions on how to run multimodal classification experiments provided in our paper.

Install conda environment and torch
```
conda create -n d360
conda activate d360
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
```

We will also use foxglove to get the data:
```
pip install mcap-protobuf-support==0.0.10 mcap-ros2-support==0.1.0 mcap-ros1-support==0.4.0 
pip install foxglove-data-platform==0.6.0 duckdb ipympl 
pip install tensorboardX tyro opencv-python
```

Generate Data. It fetches foxglove data to `data/` folder.

```
python tools/gen_data_from_foxglove.py
```

Then one can run modality according to the following example script:
```
python train.py --data-path data/cls --rgb --output output_rgb
python train.py --data-path data/cls --imu --output output_imu
python train.py --data-path data/cls --mic --output output_mic  # simple vector form
python train.py --data-path data/cls --specgram --output output_specgram
```

To run experiments with multiple modalities, simply append the arguments together
```
CUDA_VISIBLE_DEVICES=0 python train.py --data-path data/cls --rgb --imu --mic --specgram --output output_name
```
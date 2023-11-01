## VINS-Mask
A ROI-mask Feature Tracker for Monocular Visual-Inertial System, ICARCE 2022.

#### [Project Page](https://sunjiayuanro.github.io/vins_mask/)

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2 **CUDA and pytorch**
Follow [Pytorch Installation](https://github.com/pytorch/pytorch#installation).

1.3 **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **sudo make install**.

1.4 **gRPC and Protocol Buffers**
Follow [gRPC Installation](https://grpc.io/docs/languages/cpp/quickstart/), remember to **sudo make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.3.1, Protobuf 3.11.2, gRPC 1.9.0, torch 1.8.2, CUDA 11.1)

## 2. Build VINS-Mask on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/sunjiayuanro/VINS-Mask.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. VINS-Mask on Public datasets
Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). We only use one camera.

3.1 Run feature extraction service.
```
    # If you have an Anaconda environment
    conda activate YOUR_ENV
    # Run feature extraction service
    cd ~/catkin_ws/src/VINS-Mask/deep_feature
    python feat_service.py
```

3.2 Launch the vins_estimator, rviz and play the bag file respectively. Take MH_01 for example
```
    roslaunch vins_estimator euroc_no_extrinsic_param.launch
    roslaunch vins_estimator vins_rviz.launch
    rosbag play YOUR_PATH_TO_DATASET/MH_01_easy.bag 
```
3.3 More helpful tutorials can be found in [VIMS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).

## 4. Citation
```
@INPROCEEDINGS{vinsmask,
  author={Sun, Jiayuan and Song, Fangwei and Ji, Luping},
  booktitle={2022 International Conference on Automation, Robotics and Computer Engineering (ICARCE)}, 
  title={VINS-Mask: A ROI-mask Feature Tracker for Monocular Visual-inertial System}, 
  year={2022}
}
```

## 5. Acknowledgements
Thanks for [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).

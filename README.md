# [IROS2025] Ground-Fusion++: A Resilient Modular Multi-Sensor Fusion SLAM Framework

💎 Project Lead: [**Jie Yin 殷杰**](https://sjtuyinjie.github.io/) &emsp; 
🌐 [[Website]](https://sjtuyinjie.github.io/M3DGR-website/) &emsp; 
📝 [[Paper]](https://arxiv.org/abs/2507.08364) &emsp; 
➡️ [[Dataset]](https://github.com/sjtuyinjie/M3DGR) &emsp; 
⭐️ [[Pre Video]](TBD) &emsp; 
🔥 [[News]](https://mp.weixin.qq.com/s/2dVvuS3z6YDXbCG9-EOYuw)

[![Author](https://img.shields.io/badge/Author-Jie%20Yin-blue)](https://sjtuyinjie.github.io/)
[![Website](https://img.shields.io/badge/Website-M3DGR--web-green)](https://sjtuyinjie.github.io/M3DGR-website/)
[![Paper](https://img.shields.io/badge/Paper-2507.08364-yellow)](https://arxiv.org/abs/2507.08364)
[![Dataset](https://img.shields.io/badge/Dataset-M3DGR-red)](https://github.com/sjtuyinjie/M3DGR)
[![stars](https://img.shields.io/github/stars/sjtuyinjie/Ground-Fusion2.svg)](https://github.com/sjtuyinjie/Ground-Fusion2)
[![forks](https://img.shields.io/github/forks/sjtuyinjie/Ground-Fusion2.svg)](https://github.com/sjtuyinjie/Ground-Fusion2)
[![open issues](https://img.shields.io/github/issues-raw/sjtuyinjie/Ground-Fusion2)](https://github.com/sjtuyinjie/Ground-Fusion2/issues)
[![closed issues](https://img.shields.io/github/issues-closed-raw/sjtuyinjie/Ground-Fusion2)](https://github.com/sjtuyinjie/Ground-Fusion2/issues)

**Core contributors:** Deteng Zhang, Junjie Zhang, Yihong Tian, Jie Yin*(Project Lead)

---

## Notice 📢
### News
**2025.06.16:** Our paper has been accepted to IROS 2025!  
All datasets and code will be released soon — stay tuned!



### TODO
- [x] Release camera-ready version of IROS2025 paper.[[paper](https://arxiv.org/abs/2507.08364)]
- [x] Release 40 SLAM codes adapted for M3DGR dataset.[[codes](https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#6-supported-slam-algorithm-list)]
- [x] Release Ground-Fusion++ code, with examples on M3DGR on M2DGR-plus. [[code](https://github.com/sjtuyinjie/Ground-Fusion2)]
- [x] Release most sequences in the paper included with GT and calibration files to make sure all results can be reproduced.[[data](https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#5-dataset-sequences)]
- [ ] Release long-term sequences upon our journal paper acception.
- [ ] Release a much more competitive and robust SLAM system upon our journal paper acception. Please look forward to our ongoing research!

> 🔍 For those interested in accessing the unreleased M3DGR sequences in advance, we recommend first thoroughly evaluating your methods on the already released sequences. After that, feel free to contact us at **zhangjunjie587@gmail.com** to request early access for research purposes.


## 1. Introduction 🎯

This repository contains the official implementation of our **IROS 2025** paper:

> **"Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and a Resilient Framework"**

In this work, we propose a complete solution for robust SLAM on ground robots operating under degraded conditions. Our key contributions are:

-  **[M3DGR Benchmark](https://github.com/sjtuyinjie/M3DGR)**: A challenging multi-sensor, multi-scenario SLAM benchmark dataset with systematiclly induced degradation. 
-  **Ground-Fusion++ ([Link](https://github.com/sjtuyinjie/Ground-Fusion2))**): A resilient and modular SLAM framework integrating heterogeneous sensors for robust localization and high-quality mapping.
-  **Comprehensive Evaluation([Link](https://github.com/sjtuyinjie/M3DGR/tree/main/baseline_systems))**: A comprehensive evaluation of over 40 cutting-edge SLAM methods on M3DGR.






## 2. Key Features 🔧

- **Multi-sensor Integration:** GNSS, RGB-D camera, IMU, wheel odometer, and LiDAR.  
- **Robustness:** Combines state-of-the-art SLAM components to ensure accurate localization and mapping in large-scale, real-world scenarios.  
- **Modularity:** Designed as an extensible baseline to support future research and practical deployments in complex environments.

<div align="center">
  <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/mesh.gif" width="800px" alt="Ground-Fusion++ Demo">
</div>

## 3. Prerequisites and Installation

### 3.1 Ubuntu and ROS
Tested on Ubuntu 20.04(with ROS Noetic and OpenCV4).

### 3.2 Prerequisite
This package requires [Eigen 3.3.7](https://github.com/PX4/eigen), [Ceres 1.14](https://ceres-solver.googlesource.com/ceres-solver),[Sophus](https://github.com/strasdat/Sophus.git ), Sophus_no_template, fmt and Livox-SDK. We provide thridparty folder with all the third-party libraries, you can [download](https://drive.google.com/file/d/1umjPqhYcBjMMFPogh5NlPa7JLTkwftNW/view?usp=drive_link) it and install:

~~~
cd thirdparty
sudo chmod +x ./thirdparty/install.sh
sudo ./thirdparty/install.sh
~~~

### 3.3 Build Ground-Fusion++
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/Ground-Fusion2.git
cd ../..
catkin_make
```

## 4. Running with Docker
We provide a Dockerfile so you can easily replicate our setup. Below are the steps to build the Docker image.

- Install docker and nvidia-docker2. You can find tutorials like [this](https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/Ground-Fusion%2B%2B/docker/readme.md).
   
-  Pull the ROS image in advance.
   ```
   sudo systemctl start docker
   sudo docker pull j3soon/ros-noetic-desktop-full
   ```
   
-  Pull and build the Docker image.
   ```
   cd ~/catkin_ws/src
   git clone https://github.com/sjtuyinjie/Ground-Fusion2.git
   cp Ground-Fusion++/docker/{Dockerfile,ros.asc} ..
   cd ..
   wget 'https://drive.google.com/uc?id=1umjPqhYcBjMMFPogh5NlPa7JLTkwftNW&export=download' && unzip *.zip -d . && rm *.zip
   sudo docker build -t groundfusion2 .
   ```
   
-  Start the container and mount the data directory.
   ```
   sudo xhost +local:docker
   
   sudo docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri \
    -v ./data:/root/data \    
    groundfusion2 /bin/bash
   ```
   Change the ```./data``` to the path where the rosbag is actually stored.
   
-  Running.
   
   The one container terminal and type:
   ```
   cd /root/ws
   export MESA_GL_VERSION_OVERRIDE=3.3
   source devel/setup.bash
   roslaunch groundfusion2 run_m3dgr.launch
   ```
   Open another container terminal:
   ```
   cd /root/data
   rosbag play Dynamic01.bag
   ```

[Here](https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/Ground-Fusion%2B%2B/docker/readme.md) you can find a more detailed docker build tutorial.

## 5. Run Examples 🚀
### 5.1 M3DGR dataset
Download [M3DGR](https://github.com/sjtuyinjie/M3DGR) dataset and give a star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch groundfusion2 run_m3dgr.launch
(roslaunch groundfusion2 run_m3dgr_avia.launch # Use AVIA)
~~~
<div align=center>
<table>
  <tr>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/Outdoor01_mesh.gif" width="370px" height="280px">
      <div> <a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#51-standard">Outdoor01</a><div>
    </td>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/Outdoor04_mesh.gif" width="330px" height="220px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#51-standard">Outdoor04</a></div>
    </td>
  <tr>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/Dark01_mesh.gif" width="400px" height="300px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#outdoor">Dark01</a></div>
    </td>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/mesh6.gif" width="400px" height="300px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#outdoor-1">Grass02</a></div>
    </td>
</table>
</div>

### 5.2 M2DGR-Plus dataset
Download [M2DGR-Plus](https://github.com/sjtuyinjie/M2DGR-plus) dataset and give a star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch groundfusion2 run_m2dgrp.launch
~~~

### 5.3 You can use rviz to view the trajectory switching
If you want to see the switching situation or the mesh quality is poor, you can run the following command to check which subsystem has the problem. Especially when you use AVIA lidar indoors, this often happens.

<div align=center>
<table>
  <tr>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/switch_path.gif" width="350px" height="270px">
      <div>Corridor01* of M3DGR, use MID360</div>
    </td>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/switch_path3.gif" width="350px" height="220px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#indoor">Dynamic01</a> of M3DGR, use AVIA</div>
    </td>
  </tr>
  <tr>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/switch_path5.gif" width="350px" height="220px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#53-lidar-degeneration-">Elevator01</a> of M3DGR, use MID360</div>
    </td>
    <td align="center" width="400px">
      <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/switch_path4.gif" width="350px" height="220px">
      <div><a href="https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#53-lidar-degeneration-">Elevator01</a> of M3DGR, use AVIA</div>
    </td>
  </tr>
</table>
</div>

**🔵Blue** is the Fusion path of Ground-Fusion++, **🟢Green** is the LIO submodule path, and **🔴Red** is the VIO submodule path (*: This sequence comes from Table VIII of the [paper](https://arxiv.org/abs/2507.08364)).

~~~
# [launch] open a terminal and type:
source devel/setup.bash
rosrun rviz rviz -d $(rospack find groundfusion2)/launch/rviz.rviz
~~~



> ⚠️ **Known Issues**:  
> - In most sequences, our provided configurations can directly reproduce the results reported in the paper. However, in certain cases, parameter fine-tuning may be required for optimal performance
> - The mapping thread is relatively computationally intensive and may require a good machine to run smoothly.
> - 💡Our team is actively working on a next-generation version of Ground-Fusion++. Stay tuned for updates and follow our latest research!!



## 6. Citation 📄

```bibtex
@article{zhang2025towards,
  title={Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and A Resilient Framework},
  author={Zhang, Deteng and Zhang, Junjie and Sun, Yan and Li, Tao and Yin, Hao and Xie, Hongzhao and Yin, Jie},
  journal={arXiv preprint arXiv:2507.08364},
  year={2025}
}

@inproceedings{yin2024ground,
  title={Ground-fusion: A low-cost ground slam system robust to corner cases},
  author={Yin, Jie and Li, Ang and Xi, Wei and Yu, Wenxian and Zou, Danping},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={8603--8609},
  year={2024},
  organization={IEEE}
}
@article{yin2021m2dgr,
  title={M2dgr: A multi-sensor and multi-scenario slam dataset for ground robots},
  author={Yin, Jie and Li, Ang and Li, Tao and Yu, Wenxian and Zou, Danping},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={2266--2273},
  year={2021},
  publisher={IEEE}
}

```

## 7. Star History ⭐️

[![Star History Chart](https://api.star-history.com/svg?repos=sjtuyinjie/Ground-Fusion2&type=Timeline)](https://star-history.com/#Ashutosh00710/github-readme-activity-graph&Timeline)

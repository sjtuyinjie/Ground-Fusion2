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

**2025.06.16:** Our paper has been accepted to IROS 2025!  
All datasets and code will be released soon — stay tuned!

<div align="center">
  <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/demo.gif" width="500px" alt="Ground-Fusion++ Demo">
</div>


## 1. Introduction 🎯

This repository contains the official implementation of our **IROS 2025** paper:

> **"Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and a Resilient Framework"**

In this work, we propose a complete solution for robust SLAM on ground robots operating under degraded conditions. Our key contributions are:

-  **[M3DGR Benchmark](https://github.com/sjtuyinjie/M3DGR)**: A challenging multi-sensor, multi-scenario SLAM benchmark dataset with systematiclly induced degradation. 
-  **Ground-Fusion++ ([Link](https://github.com/sjtuyinjie/Ground-Fusion2))**): A resilient and modular SLAM framework integrating heterogeneous sensors for robust localization and high-quality mapping.
-  **Comprehensive Evaluation([Link](https://github.com/sjtuyinjie/M3DGR/tree/main/baseline_systems))**: A comprehensive evaluation of over 40 cutting-edge SLAM methods on M3DGR.



### TODO
- [x] Release camera-ready version of IROS2025 paper.[[paper](https://arxiv.org/abs/2507.08364)]
- [x] Release 40 SLAM codes adapted for M3DGR dataset.[[codes](https://github.com/sjtuyinjie/M3DGR?tab=readme-ov-file#6-supported-slam-algorithm-list)]
- [ ] Release Ground-Fusion++ code, with examples on M3DGR on M2DGR-plus.
- [ ] Release half of M3DGR sequences (all 10 evaluated representative sequences in the paper included) with GT and calibration files to make sure all results can be reproduced.
- [ ] Release another half of M3DGR sequences upon our journal paper acception.
- [ ] Release a much more competitive and robust SLAM system upon our journal paper acception.

> 🔍 For those interested in accessing the unreleased M3DGR sequences in advance, we recommend first thoroughly evaluating your methods on the already released sequences. After that, feel free to contact us at **zhangjunjie587@gmail.com** to request early access for research purposes.


## 2. Key Features 🔧

- **Multi-sensor Integration:** GNSS, RGB-D camera, IMU, wheel odometer, and LiDAR.  
- **Robustness:** Combines state-of-the-art SLAM components to ensure accurate localization and mapping in large-scale, real-world scenarios.  
- **Modularity:** Designed as an extensible baseline to support future research and practical deployments in complex environments.
  
## 3. Compile
```
cd ~/catkin_ws/src
git clone https://github.com/sjtuyinjie/Ground-Fusion2.git
cd ../..
catkin_make
```


## 4. Run Examples 🚀
### 4.1 M3DGR dataset
Download [M3DGR](https://github.com/sjtuyinjie/M3DGR) dataset and give a star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch groundfusion2 run_m3dgr.launch
~~~


### 4.2 M2DGR-Plus dataset
Download [M2DGR-Plus](https://github.com/sjtuyinjie/M2DGR-plus) dataset and give a star.


~~~
# [launch] open a terminal and type:
source devel/setup.bash
roslaunch groundfusion2 run_m2dgrp.launch
~~~

### 4.3 You can use rviz to view the trajectory switching
~~~
# [launch] open a terminal and type:
source devel/setup.bash
rosrun rviz rviz -d $(rospack find groundfusion2)/launch/rviz.rviz
~~~



> ⚠️ **Known Issues**:  
> - In most sequences, our provided configurations can directly reproduce the results reported in the paper. However, in certain cases, parameter fine-tuning may be required for optimal performance
> - The mapping thread is relatively computationally intensive and may require a good machine to run smoothly.
> - 💡Our team is actively working on a next-generation version of Ground-Fusion++. Stay tuned for updates and follow our latest research!!


## ⭐️ 5. Star History

[![Star History Chart](https://api.star-history.com/svg?repos=sjtuyinjie/Ground-Fusion2&type=Timeline)](https://star-history.com/#Ashutosh00710/github-readme-activity-graph&Timeline)

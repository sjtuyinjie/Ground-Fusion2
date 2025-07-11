# [IROS2025] Ground-Fusion++: A Resilient Modular Multi-Sensor Fusion SLAM Framework

💎 Corresponding Author: [**Jie Yin 殷杰**](https://sjtuyinjie.github.io/) &emsp; 📝 [[Paper]](https://github.com/sjtuyinjie/M3DGR/blob/main/_IROS2025_GroundFusion2_M3DGR.pdf) &emsp; ➡️ [[Dataset]](https://github.com/sjtuyinjie/M3DGR) &emsp; ⭐️ [[Presentation Video]](TBD) &emsp; 🔥 [[News]](TBD)

**Main contributors:** Deteng Zhang, Junjie Zhang, Yan Sun, Yihong Tian, Jie Yin*(Project Lear)

---

## 📢 Notice

**2025.06.16:** Our paper has been accepted to IROS 2025!  
All datasets and code will be released soon — stay tuned!

<div align="center">
  <img src="https://github.com/sjtuyinjie/Ground-Fusion2/blob/main/fig/demo.gif" width="500px" alt="Ground-Fusion++ Demo">
</div>


## 🎯 Introduction

This repository contains the official implementation of our **IROS 2025** paper:

> **"Towards Robust Sensor-Fusion Ground SLAM: A Comprehensive Benchmark and a Resilient Framework"**

In this work, we propose a complete solution for robust SLAM on ground robots operating under degraded conditions. Our key contributions are:

- 📦 **[M3DGR Benchmark](https://github.com/sjtuyinjie/M3DGR)**: A comprehensive multi-sensor, multi-scenario SLAM benchmark for evaluating performance in challenging environments.  
- 🚀 **Ground-Fusion++ (this repo)**: A resilient and modular SLAM framework integrating heterogeneous sensors for robust localization and high-quality mapping.


## TODO
- [x] Release camera-ready version paper.[[paper](https://github.com/sjtuyinjie/M3DGR/blob/main/_IROS2025_GroundFusion2_M3DGR.pdf)]
- [ ] Release Ground-Fusion++ code.
- [ ] Release M3DGR dataset with GT and calibration files.
- [ ] Release 40 SLAM codes adapted for M3DGR dataset.


## 🔧 Key Features

- **Multi-sensor Integration:** GNSS, RGB-D camera, IMU, wheel odometer, and LiDAR.  
- **Robustness:** Combines state-of-the-art SLAM components to ensure accurate localization and mapping in large-scale, real-world scenarios.  
- **Modularity:** Designed as an extensible baseline to support future research and practical deployments in complex environments.
  



## 🚀 Run Examples
### M3DGR dataset

### M3DGR dataset

> ⚠️ **Known Issues**:  
> - In most sequences, our provided configurations can directly reproduce the results reported in the paper. However, in certain cases, parameter fine-tuning may be required for optimal performance
> - The mapping thread is relatively computationally intensive and may require a good machine to run smoothly.
> - 💡Our team is actively working on a next-generation version of Ground-Fusion++. Stay tuned for updates and follow our latest research!!


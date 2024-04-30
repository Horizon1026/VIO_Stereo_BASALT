# VIO_Stereo_BASALT
A simple visual-inertial odometry reconstructed from basalt.

# Components
- [x] Data manager.
- [x] Data loader.
- [x] Frontend.
    - [x] Stereo visual frontend.
- [x] Backend.
    - [x] Backend initialization.
    - [x] Backend optimization.
    - [x] Backend marginalization.
- [x] Log record.

# Dependence
- Slam_Utility
- Feature_Detector
- Feature_Tracker
- Sensor_Model
- Vision_Geometry
- Image_Processor
- Slam_Solver
- Visual_Frontend
- Binary_Data_Log
- Visualizor2D
- Visualizor3D

# Tips
- 这是为了学习开源的 BASALT-VIO 而创建的用于复现/魔改 paper 的代码仓库，欢迎一起交流学习，不同意商用；
- 初始化过程导入了贺博开源的 VIO 鲁棒初始化；
- 实际 pipeline 和 BASALT 可能有较大出入，参考了部分 VINS-Fusion 的思想；
- 代码目前还有点问题，暂时找不到原因，后续找到会再做修复；

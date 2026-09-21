# 深度单位证据及边界

本地dpkg版本：ros-jazzy-realsense2-camera 4.58.1-1noble.20260615.150439；本地package.xml版本4.58.1。

核对官方同版本上游源码：
https://raw.githubusercontent.com/IntelRealSense/realsense-ros/4.58.1/realsense2_camera/src/base_realsense_node.cpp

`fix_depth_scale`中关键语句：

```cpp
static const float meter_to_mm = 0.001f;
p_to[j] = p_from[j] * _depth_scale_meters / meter_to_mm;
```

深度帧经fillCVMatImageAndReturnStatus时调用fix_depth_scale，之后转ROS图像发布。本轮两路实际encoding均16UC1，分析按ROS毫米输出解释。

这是本地包版本与上游同tag实现的源码核对，不是对本地二进制或设备尺度的独立实测。未获取SDK实时depth_units，未用已知长度校准绝对测距。原始整数.npy保留，后续可更改尺度解释重新计算。

同一深度帧的raw/aligned数值近一致与该解释相容，但一致性本身不能证明绝对毫米精度。

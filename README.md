<!--
 * @Author: your name
 * @Date: 2019-11-05 21:41:24
 * @LastEditTime: 2020-02-27 15:18:19
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /gx2019_omni_simulations/README.md
 -->
# autonomus transport industrial system

|Author|@lifuguan , @Wacokgde , @Yanwu Chen|
|---|---
|ROS|melodic
|Compute platform| Intel Core I3-7100U|date 2019.11.3

![Our car.](img/car.jpg)
![Our car.](img/car2.jpg)
![Our car.](img/car3.jpg)

## CONSTRUCT

1. **utility.h**

- `utility` 类用于收集各种功能性函数
    - `GetEuclideanDistance()` 得到两个pose点之间的距离
    - `GetYawFromOrientation()` 将四元数转换成欧拉角

2. **PoseDrawer.h**
- `PoseDrawer` 类用于收集关于PoseStamped的功能性函数  


## TODO
**utility.h** 文件中的 `TransformPose()` 函数


<!--
 * @Author: your name
 * @Date: 2019-11-05 21:41:24
 * @LastEditTime: 2020-07-22 10:35:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/README.md
 -->
# autonomus transport industrial system

|Author|@lifuguan , @Wacokgde , @zhuangzibo|
|---|---
|ROS|melodic
|Compute platform| Intel Core I3-7100U|date 2019.11.3

![MAP](figures/map.png)

## URDF model
1. `car_.urdf`：原版小车（四轮驱动，体型小）
2. `robot.urdf`：新版小车（两轮驱动，只有一层）
3. `robot1.urdf`：新版小车（两轮驱动，三层结构）

## Execute file

1. **test**
- `test` 是一个测试文件，目前为主程序（2020.5.12）
  
2. **netComModule**
- `netComModule` 专门用于与控制系统进行TCP/IP通信，用于发送实时位置、接收目标位置

## Head file

1. **utility.h**

- `utility` 类用于收集各种功能性函数
    - `GetEuclideanDistance()` 得到两个pose点之间的距离
    - `GetYawFromOrientation()` 将四元数转换成欧拉角

2. **PoseDrawer.h**
- `PoseDrawer` 类用于收集关于PoseStamped的功能性函数  

3. **NetworkCom.h**
- `NetworkCom` 是一个基于面向对象的TCP/IP异步通信的类


<!--
 * @Author: your name
 * @Date: 2020-07-20 18:07:28
 * @LastEditTime: 2020-07-20 18:10:31
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /autonomus_transport_industrial_system/note.md
--> 


# 个人笔记

## 关于AMCL的注意事项

- Navigation模块只从*amcl_pose*获得瞬时速度并参与后续计算
- 从*tf*处获得odom和漂移
- 所以，*amcl_pose*并不需要反馈到前面的位姿处
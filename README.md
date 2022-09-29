<!--
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-29 10:39:22
 * @FilePath: /test_ws/src/lidar_camera_projection/README.md
 * @Description: 
 * 
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved. 
-->
# lidar_camera_projection
A ROS package which projecting lidar's pointcloud to camera's image.

## TEST

## TODO

1. 优化代码，降低时间
2. 考虑IMU

## Changelog

### 2022-9-26

1. 设置3种模式，包括直接投影，yolov5识别框投影，点云聚类投影。
2. 手动滤波改为pcl直通滤波，耗时从31ms降为22ms。
3. 修复bug，调整类成员。

### 2022-9-27

1. 添加发布点云过滤数据。
5. 分离点云处理函数。

### 2022-9-28

1. 完成点云聚类投影逻辑。
2. 修改函数模板。
3. 完成tf订阅。
4. 完成对nuscenes bag的正确投影。
5. 添加Settings.h。
6. 优化投影颜色。

### 2022-9-28

1. 增加对XYZIR点云类型的支持。
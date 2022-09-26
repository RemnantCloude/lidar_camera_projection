<!--
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-26 22:08:57
 * @FilePath: /test_ws/src/lidar_camera_projection/README.md
 * @Description: 
 * 
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved. 
-->
# lidar_camera_projection
A ROS package which projecting lidar's pointcloud to camera's image.

## TEST

3. 欧几里德聚类
4. 标出3D框
5. 2D框显示优化
6. 投影颜色绘制
7. 测试AABB和OBB的速度

## TODO

1. 优化代码，降低时间
2. 订阅tf关系
3. 考虑IMU
4. XYZI->XYZIRT

## Changelog

1. 设置3种模式，包括直接投影，yolov5识别框投影，点云聚类投影。
2. 手动滤波改为pcl直通滤波，耗时从31ms降为22ms。
3. 修复bug，调整类成员。
4. 添加发布点云过滤数据。
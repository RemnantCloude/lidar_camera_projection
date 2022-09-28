<!--
 * @Author: RemnantCloude remnantcloude@gmail.com
 * @Date: 2022-09-10 09:45:11
 * @LastEditors: RemnantCloude remnantcloude@gmail.com
 * @LastEditTime: 2022-09-28 11:10:51
 * @FilePath: /test_ws/src/lidar_camera_projection/README.md
 * @Description: 
 * 
 * Copyright (c) 2022 by RemnantCloude remnantcloude@gmail.com, All Rights Reserved. 
-->
# lidar_camera_projection
A ROS package which projecting lidar's pointcloud to camera's image.

## TEST

1. 2D框显示优化
2. 投影颜色绘制

## TODO

1. 优化代码，降低时间
2. 考虑IMU
3. XYZI->XYZIRT

## Changelog

1. 设置3种模式，包括直接投影，yolov5识别框投影，点云聚类投影。
2. 手动滤波改为pcl直通滤波，耗时从31ms降为22ms。
3. 修复bug，调整类成员。
4. 添加发布点云过滤数据。
5. 分离点云处理函数。
6. 完成点云聚类投影逻辑。
7. 修改函数模板。
8. 完成tf订阅。
9. 完成对nuscenes bag的正确投影。
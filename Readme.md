# 基于激光地图的雷达到IMU标定工具
## 1.docker 版本
```
cuidarchan/test_v1.0:v2.0
```
## 2.操作指南
### 2.1 拉取docker镜像，并进入
```
bash docker_pull.sh(仅第一次运行使用)
bash docker_start.sh
```
### 2.2 登入可视化docker界面，
进入docker后，有提示，默认网址如下所示，密码为vncpasswd  
```
http://127.0.0.1:6901 
```
### 2.3 编译节点
在网页内打开命令行，执行编译指令
```
catkin_make
```

### 2.4运行节点
```
source devel/setup.bash
roslaunch lidar_imu_calib_based_on_map align_to_map.launch
rosbag play src/lidar_IMU_calib_based_on_map/data/data_example.bag -l
```

### 2.5 显示结果
如images下的图片，可视化为标定匹配结果，右侧命令行显示最终的标定外参。
![Image](https://github.com/cuiDarchan/lidar_IMU_calib_project/blob/main/images/result.png)  

## 3.基本原理
1）预制一块点云地图，将车辆静止放入场景中，获取位置与姿态，将点云地图转化为IMU坐标系下。  
2）利用想要标定的激光外参初值，将点云数据进行坐标变换，转换到IMU坐标系下。  
3）相同IMU坐标系下，点云与点云地图进行NDT匹配，每次匹配所得到的结果，作为下次匹配的推测值。  
4）最后发布匹配点云和点云地图的融合效果，Rviz里面显示观察重合程度，外参结果在命令行下有打印信息。  

## 4.参考博文
https://adamshan.blog.csdn.net/article/details/105930565

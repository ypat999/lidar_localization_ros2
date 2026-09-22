# lidar_localization_ros2
A ROS2 package of 3D LIDAR-based Localization.

> ## XTDrone2 fork 扩展：原点基准精准降落（Origin Baseline Landing）
>
> 本 fork 在原版全局图定位之上新增"起飞点基准"能力，`map->odom` 静态 TF 由本节点独家持有
> （Super-LIO launch 已停用恒等 map->odom 发布）。工作时序：
>
> 1. **启动攒基准**：飞机在坪上（xy < `origin_baseline_radius`）时积累
>    `origin_baseline_frames` 帧点云，按 map->`origin_baseline_base_frame` 位姿
>    全分辨率合并为原点基准地图（不降采样；可落盘 `origin_baseline_pcd_path` 检查）。
>    积累期间**挂起全局图初始定位**（先基准、后定位）。
> 2. **起飞武装**：基准就绪后须首次离圈（真实起飞）才激活降落匹配，坪上待机期间
>    全局图定位保持权威（防止启动即被基准误接管）。
> 3. **返航接管**：xy 重新进圈后，每 `origin_baseline_match_interval`（默认1s）将
>    **当前单帧**点云变换到 map 系与基准直接 GICP 匹配；fitness ≤
>    `origin_baseline_score_threshold` 且优于进圈后历史最低误差时才更新 `map->odom`
>    静态 TF（最低误差锁定）。突变护栏：与当前 map->odom 差 >30° 或 >1m 拒绝并 WARN。
> 4. **出圈恢复**：离圈即回到全局图定位流程，下次起飞重新进圈重新匹配。
>
> 新增参数（`localization.yaml`）：
>
> | 参数 | 默认 | 说明 |
> |---|---|---|
> | `enable_origin_baseline` | true | 原点基准精准降落总开关 |
> | `origin_baseline_frames` | 10 | 基准积累帧数 |
> | `origin_baseline_radius` | 1.5 | 原点触发半径（米，xy） |
> | `origin_baseline_match_interval` | 1.0 | 进圈后匹配周期（秒） |
> | `origin_baseline_score_threshold` | 1.0 | 降落匹配 fitness 上限，超过丢弃 |
> | `origin_baseline_base_frame` | base_link | 基准锚定系（与航点/控制点 `/lio/robo/odom` 一致） |
> | `origin_baseline_pcd_path` | /tmp/origin_baseline.pcd | 基准地图落盘路径（空=禁用） |
>
> 实测检查点：`Origin baseline ready` / `armed: left takeoff zone` /
> `Entered origin landing zone` / `Landing baseline NEW BEST`；
> 出现 `rejected as jump` 说明运行时 /tf 有瞬时异常，需查 `/tf_static` 发布者名单。


<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

## Requirements

- [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2.git)

## IO
- input  
/cloud  (sensor_msgs/PointCloud2)  
/map  (sensor_msgs/PointCloud2)  
/initialpose (geometry_msgs/PoseStamed)(when `set_initial_pose` is false)  
/odom (nav_msgs/Odometry)(optional)   
/imu  (sensor_msgs/Imu)(optional)  

- output  
/pcl_pose (geometry_msgs/PoseStamped)  
/path (nav_msgs/Path)  
/initial_map (sensor_msgs/PointCloud2)(when `use_pcd_map` is true)  

## params

|Name|Type|Default value|Description|
|---|---|---|---|
|registration_method|string|"NDT_OMP"|"NDT" or "GICP" or "NDT_OMP" or "GICP_OMP"|
|score_threshold|double|2.0|registration score threshold|
|ndt_resolution|double|2.0|resolution size of voxels[m]|
|ndt_step_size|double|0.1|step_size maximum step length[m]|
|ndt_num_threads|int|4|threads using NDT_OMP(if `0` is set, maximum alloawble threads are used.)|
|transform_epsilon|double|0.01|transform epsilon to stop iteration in registration|
|voxel_leaf_size|double|0.2|down sample size of input cloud[m]|
|scan_max_range|double|100.0|max range of input cloud[m]|
|scan_min_range|double|1.0|min range of input cloud[m]|
|scan_periad|double|0.1|scan period of input cloud[sec]|
|use_pcd_map|bool|false|whether pcd_map is used or not|
|map_path|string|"/map/map.pcd"|pcd_map or ply_map file path|
|set_initial_pose|bool|false|whether or not to set the default value in the param file|
|initial_pose_x|double|0.0|x-coordinate of the initial pose value[m]|
|initial_pose_y|double|0.0|y-coordinate of the initial pose value[m]|
|initial_pose_z|double|0.0|z-coordinate of the initial pose value[m]|
|initial_pose_qx|double|0.0|Quaternion x of the initial pose value|
|initial_pose_qy|double|0.0|Quaternion y of the initial pose value|
|initial_pose_qz|double|0.0|Quaternion z of the initial pose value|
|initial_pose_qw|double|1.0|Quaternion w of the initial pose value|
|use_odom|bool|false|whether odom is used or not for initial attitude in point cloud registration|
|use_imu|bool|false|whether 9-axis imu is used or not for point cloud distortion correction|
|enable_debug|bool|false|whether debug is done or not|
|enable_timer_publishing|bool|false|if true, publish tf and pose on a set timer frequency|
|pose_publish_frequency|double|10.0|publishing frequency if enable_timer_publishing is true|

## demo

demo data(ROS1) by Tier IV（The link has changed and is now broken.)  
https://data.tier4.jp/rosbag_details/?id=212  
To use ros1 rosbag , use [rosbags](https://pypi.org/project/rosbags/).  
The Velodyne VLP-16 was used in this data.

Before running, put `bin_tc-2017-10-15-ndmap.pcd` into your `map` directory and  
edit the `map_path` parameter of `localization.yaml` in the `param` directory accordingly.
```
rviz2 -d src/lidar_localization_ros2/rviz/localization.rviz
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
ros2 bag play tc_2017-10-15-15-34-02_free_download/
```

<img src="./images/path.png" width="640px">

Green: path, Red: map  
(the 5x5 grids in size of 50m × 50m)

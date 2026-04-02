
ros2 launch realsense2_camera rs_launch.py pointcloud.enable:=true align_depth.enable:=true


ros2 launch force_admittance_servo force_admittance_servo.launch.py 
ros2 launch moveit_servo jaka_servo_example.launch.py


ros2 launch path_servo_control servo_control.launch.py 
ros2 service call /path_servo/start std_srvs/srv/Trigger "{}"
ros2 service call /path_servo/stop std_srvs/srv/Trigger "{}"





# 1. 一键启动全系统（含 RViz2）
ros2 launch point_cloud_processor processor_launch.py

# 2. 开始采集点云
ros2 service call /pcd_saver/set_saving \
  point_cloud_processor/srv/SetSaving \
  "{enable: true, save_dir: 'data', max_frames: 1}"

# 步骤 1：裁剪
ros2 service call /crop_point_cloud \
  point_cloud_processor/srv/CropPointCloud \
  "{}"

# 步骤 2：预处理重建
ros2 action send_goal /process_point_cloud \
  point_cloud_processor/action/ProcessPointCloud \
  "{input_pcd_path: '', output_ply_path: ''}" --feedback

# 步骤 3：路径规划
ros2 action send_goal /plan_coverage_path \
  point_cloud_processor/action/PlanCoveragePath \
  "{input_ply_path: '', output_csv_path: '', output_gcode_path: ''}" --feedback


# 4. 规划完成后热重载可视化（无需重启节点）
ros2 service call /coverage_visualizer/reload std_srvs/srv/Trigger
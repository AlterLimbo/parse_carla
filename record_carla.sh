rostopic pub /carla/ego_vehicle/enable_autopilot std_msgs/Bool "data: true" -1

rosbag record /carla/ego_vehicle/camera/rgb/front/camera_info /carla/ego_vehicle/camera/rgb/front/image_color /carla/ego_vehicle/camera/rgb/view/camera_info /carla/ego_vehicle/camera/rgb/view/image_color /carla/ego_vehicle/gnss/gnss1/fix /carla/ego_vehicle/lidar/lidar1/point_cloud /carla/ego_vehicle/odometry /carla/marker /tf /tf_static -O a.bag --duration=3m

python parse_carla.py a.bag

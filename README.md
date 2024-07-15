# RACETRACK MAPPER

This ROS2 project maps a racetrack using LiDAR data and visualizes it in RViz.

# PREREQUISITES

Make sure you have the following installed:
  - ROS2
  - PCL
  - pcl_ros package

# BUILD

  1. Clone the repo
  2. colcon build
  3. Source your ROS2 WorkSpace
       source /opt/ros/humble/setup.bash
       source install/setup.bash
  4. Play the rosbag
       ros2 bag play -l 240427-10_50_19_cut_0.mcap
  5. Run the node
       ros2 run your_ros2_project map_maker_sub
  6. Run RViz2
       rviz2
  7. Inside RViz
       Add -> By topic -> racetrack/map_data/PointCloud2 -> OK
       Set the point size 0.1 or 1m (for better visibility)
     

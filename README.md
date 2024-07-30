# TechSimApiROS
##### This is a ros2 package for interaction with AgroTechAvia simulators.

---
##### The following functionality is implemented in this package:
1. Image capture from drone cameras in simulator. 
2. Recognition of aruco markers on image.
3. Obtaining point cloud from lidar. 

---

 To install the package, go to  `<ros_work_space>/src` and clone this repo. Then go to `<ros_work_space>` and use commands `colcon build` and `source ~/setup.bash`.
Then you can run the package with the command:

```
ros2 launch computer_vision_functionality driver.launch.py
```



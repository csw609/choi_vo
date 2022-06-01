# choi_vo
Stereo Visual Odometry Estimator ROS2 Package

<!-- choi_vo is stereo visual odometry estimator ROS2 package -->

## Prerequisites
- Ubuntu 20.04
- ROS2 foxy
- OpenCV

## Build
```
    cd ~/dev_ws/src
    git clone https://github.com/csw609/choi_vo.git
    cd ../
    colcon build
    source ./install/setup.bash
```

## Run
```
    ros2 launch choi_vo choi_vo_launch.py
```

### Future Work
- Make Parameter Configure File
- Extract Keyframe
- Implement Loop Closure
- use IMU


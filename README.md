# bag to video
## 1. Prerequisites
Ubuntu  16.04.   ROS Kinetic. OpenCV 3

## 2. Build the project on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/AlexWan1027/bag_to_video.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run the project with launch file
```
     roslaunch bag_to_video bag_to_video.launch
```
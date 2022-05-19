# Autonomous-UAV-Landing

This repo is for testing and implementing different controllers and trajectory generation methods for Landing on a moving platform.

```bash
cd catkin_ws/src

#for SSH
git clone git@github.com:devrajPriyadarshi/Autonomous-UAV-Landing.git

#for HTTPS
git clone https://github.com/devrajPriyadarshi/Autonomous-UAV-Landing.git
```

## Addition files
- This repo contains landing_static.world and landing_moving.world in rotors_gazebo/worlds folder.
- A firefly drone with a downwards camera (to be added soon).
To launch the autonomous landing use :
```bash
roslaunch rotors_gazebo autonomous_landing.launch #this by default loads the static world

#OR

roslaunch rotors_gazebo autonomous_landing.launch moving_base:=true #for moving platform
```

## Papers for Reading
A general manual for working with RotorS Simulator can be found in : [RotorS – A Modular Gazebo MAV Simulator Framework](https://www.researchgate.net/publication/309291237), you can read specific parts of its right now for commanding the drone using rostopics, and how to make your own controller. The latter can be done by commenting out the lee controller node of ypur launch file,
```
<node name="lee_position_controller_node" pkg="rotors_control" type="lee_position_controller_node" output="screen">
  <rosparam command="load" file="$(find rotors_gazebo)/resource/lee_controller_$(arg mav_name).yaml" />
  <rosparam command="load" file="$(find rotors_gazebo)/resource/$(arg mav_name).yaml" />
  <remap from="odometry" to="odometry_sensor1/odometry" />
</node>
 ````
 and adding your own controller node created in rotors_control package.

Task Specific Papers:
- Alireza Mohammadi, Yi Feng, Cong Zhang, Samir Rawashdeh, and Stanley Baek, [Vision-based Autonomous Landing Using an MPC-controlled Micro UAV on a Moving Platform](https://ieeexplore.ieee.org/document/9214043)
- Yi Feng, Cong Zhang, Stanley Baek, Samir Rawashdeh and Alireza Mohammadi, [Autonomous Landing of a UAV on a Moving Platform Using Model Predictive Control](https://mdpi-res.com/d_attachment/drones/drones-02-00034/article_deploy/drones-02-00034.pdf?version=1539336596)
- Aleix Paris, Brett T. Lopez, and Jonathan P. How, [Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions.](https://arxiv.org/pdf/1909.11071.pdf)
- Davide Falanga, Alessio Zanchettin, Alessandro Simovic, Jeffrey Delmerico, and Davide Scaramuzza, [Vision-based Autonomous Quadrotor Landing on a Moving Platform.](https://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf)

(You may also add more papers and materials for reference as you prefer.)

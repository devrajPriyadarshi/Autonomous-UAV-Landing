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
- A firefly drone with a downwards camera.
To launch the autonomous landing use :
```bash
roslaunch rotors_gazebo autonomous_landing.launch #this by default loads the static world

#OR

roslaunch rotors_gazebo autonomous_landing.launch world_name:=landing_moving #for moving platform
```
## Papers for Reading
- Alireza Mohammadi, Yi Feng, Cong Zhang, Samir Rawashdeh, and Stanley Baek, [Vision-based Autonomous Landing Using an MPC-controlled Micro UAV on a Moving Platform](https://ieeexplore.ieee.org/document/9214043)
- Yi Feng, Cong Zhang, Stanley Baek, Samir Rawashdeh and Alireza Mohammadi, [https://mdpi-res.com/d_attachment/drones/drones-02-00034/article_deploy/drones-02-00034.pdf?version=1539336596](https://mdpi-res.com/d_attachment/drones/drones-02-00034/article_deploy/drones-02-00034.pdf?version=1539336596)
- Aleix Paris, Brett T. Lopez, and Jonathan P. How, [Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions.](https://arxiv.org/pdf/1909.11071.pdf)
- Davide Falanga, Alessio Zanchettin, Alessandro Simovic, Jeffrey Delmerico, and Davide Scaramuzza, [Vision-based Autonomous Quadrotor Landing on a Moving Platform.](https://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf)

# Autonomous-UAV-Landing

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
- Aleix Paris, Brett T. Lopez, and Jonathan P. How, [Dynamic Landing of an Autonomous Quadrotor on a Moving Platform in Turbulent Wind Conditions.](https://arxiv.org/pdf/1909.11071.pdf)
- Davide Falanga, Alessio Zanchettin, Alessandro Simovic, Jeffrey Delmerico, and Davide Scaramuzza, [Vision-based Autonomous Quadrotor Landing on a Moving Platform.](https://rpg.ifi.uzh.ch/docs/SSRR17_Falanga.pdf)

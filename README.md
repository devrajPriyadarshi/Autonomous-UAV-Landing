# Autonomous-UAV-Landing

```bash
cd catkin_ws/src

#for SSH
git clone git@github.com:devrajPriyadarshi/Autonomous-UAV-Landing.git

#for HTTPS
git clone https://github.com/devrajPriyadarshi/Autonomous-UAV-Landing.git
```

## Addition files :
- This repo contains landing_static.world and landing_moving.world in rotors_gazebo/worlds folder.
- A firefly drone with a downwards camera.
To launch the autonomous landing use :
```bash
roslaunch rotors_gazebo autonomous_landing.launch #this by default loads the static world

#OR

roslaunch rotors_gazebo autonomous_landing.launch world_name:=landing_moving #for moving platform
```

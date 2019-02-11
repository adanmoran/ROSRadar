# ROSRadar
ROS Driver for the ARS430 Radar. Developed for the AUToronto SAE Team as part of AER1514 (University of Toronto).

Setting up with ROS
===================
This requires ROS Kinetic to be installed in `/opt/ros/kinetic/`.

Assuming you downloaded this to your home folder, you should run the following commands

1. `rosws set ~/ROSRadar/kinetic_workspace/sandbox`
2. `source ~/ROSRadar/kinetic_workspace/setup.bash`
3. `echo $ROS_PACKAGE_PATH`
4. Confirm you get something like "/home/username/ROSRadar/kinetic_workspace/sandbox:/opt/ros/kinetic/share"

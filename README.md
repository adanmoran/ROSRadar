# ars430-ros-driver
ROS Driver for the ARS430 Radar. Developed for the AUToronto SAE Team as part of AER1514 (University of Toronto). Implements version 0.1.2 of Continental Automotive's ethernet interface.

Summary
=======
There are two nodes packaged with this git repository: rosudp and ars430.
* rosudp is a node which enables a connection with the network card to access UDP datagrams. It is 
usable with any UDP-capable device. For usage with the ARS430 radar, it emits a custom "UDPMsg.msg"
to the ROS topic "rosudp/31122" (since the ARS430 emits data on UDP port 31122).
* ars430 is a node which receives data from "rosudp/31122" and unpacks the datagram into
custom status or event ROS messages, emitted to the topic "ars430/status" or "ars430/event" depending
on which type of data was received. It also converts every detection into XYZ coordinates and
emits it to a "vizualization_markers" topic for seeing points in Rviz.


You should only need to run a single rosudp node for arbitrarily many ARS430 radars. You'll need 
as many ARS430 nodes as you have radars (and will have to change the IP address of each node).

Setting up with ROS
===================
This requires ROS Kinetic to be installed in `/opt/ros/kinetic/`.

Assuming you downloaded this to your home folder, you should run the following commands

1. `source <repo_base>/kinetic_workspace/setup.bash`
2. `echo $ROS_PACKAGE_PATH`
3. Confirm you get something like "/home/username/ROSRadar/kinetic_workspace/sandbox:/opt/ros/kinetic/share"

The rosudp assumes you have an ARS430 radar with IP address `192.168.1.*` which emits to port `31122`. You will need to set your ethernet card to have the fixed IP address `192.168.1.30` with netmask `255.255.255.0` for rosudp to work. 

If your radar's IP or port is different than that above, please set your ethernet's fixed IP to match the first 3 segments of the radar's IP, then edit the file `kinetic_workspace/sandbox/rosudp/scripts/publish_upd.py` to match your ethernet's new IP address.

The ars430 node, on the other hand, assumes the radar IP is `192.168.1.2`. Please change this to match your radar's IP after fixing issues with rosudp.

Usage
=====
After setting up with ROS, you can run the following.
```sh
roscore &
rosrun rosudp publish_udp.py &
rosrun ars430 ars430.py &
```

This will run ROS, the rosudp node, and the ars430 node in the background. You should be able to see ARS430 messages
arriving by printing them out with `rostopic echo /ars430/event` or `rostopic echo /ars430/status`.

To visualize the points in Rviz, simply open rviz with `rosrun rviz rviz`. Click "Add > Markers" and points should appear
on the screen.

Whitepaper
==========
DRIVER DOCUMENTATION AND INITIAL CHARACTERIZATION OF THE ARS430 COMING SOON
Expected delivery is summer 2019. Contact me for an informal paper in the meantime.

Improvements
============
Pull-requests for improvements are welcome. 

Please send proof of your updates working with a physical ARS430.

Suggested improvements for those interested:
* Filter which removes false positives and reduces noise by using RDI data (RCS, PDh0, Variances, etc)
* Estimation, which merges data from multiple ARS430 radars. This could involve Kalman Filtering or other point cloud alignment techniques.
It should also improve noise error if there is an overlap region, but you'll have to take into account the 
radar waves interfering with each other.
* Object tracking by using relative velocities and prediction techniques
* Take IP address / port / anything else required as a command-line input (either through rosparam or otherwise, see TODO notes in publish_udp.py and ars430.py). This will enable connection to multiple UDP ports and to multiple ARS430 radars.
* Create a roslaunch file, which will start the single rosudp node on port 31122 and as many ARS430 nodes as you want. We tried this already and had errors with rosudp when switching between computers, so please ensure this works on multiple hosts and with a real ARS430.

Acknowledgements
================
Credit goes to Adan Moran-MacDonald, Navid Kayhani, Nandhini Sayee, and Vipin Karthikeyan for creating this software package.

Thanks to Prof. Tim Barfoot and Keenan Burnett for their advice as the project evolved.

# Final project work for CSCI 597F
![image](https://github.com/ScratchyPirate/CSCI-597F-Candy-Cadets/blob/main/assets/candycadet.png)

## Candy Cadets
- John Sbur (project lead)
- Natalie Gleason (developer & engineer)
- Wil Apollo Zuber (developer)

## Deliverable
A duckiebot-based robotic system that will deliver candy to a human.

## Components
- Duckiebot.
- 3D printed tray for containing candy payload.
- 3D printed landing pad for candy to be delivered to.
- Candy dispenser.

## Resources

[CSCI 597F Final Project - Google Drive](https://drive.google.com/drive/folders/1hRoMSuwKlewXq7G9C-CAv0jkp49Vuu00?usp=sharing)

### Coding the DuckieBot
- [Duckietown shell](https://github.com/duckietown/duckietown-shell)
- [More on Duckietown shell](https://docs.duckietown.com/daffy/devmanual-software/basics/development/dts-duckietown-shell.html)
- [Duckietown documentation](https://docs.duckietown.com/ente/duckietown-manual/welcome-to-the-duckietown-manual.html)
- [Setting up a Duckiebot project](https://docs.duckietown.com/ente/duckietown-manual/70-developer-manual/ros/create-new-ros-project.html)


#### ROS
- [Coding Duckietbot with ROS](https://docs.duckietown.com/ente/duckietown-manual/70-developer-manual/ros/introduction-to-using-ros-with-duckietown.html#dtproject-ros) 
- [Catkin](https://wiki.ros.org/catkin/Tutorials): Workspaces for ROS.
- [Rospy](https://wiki.ros.org/rospy): Python ROS package.

### Hardware Resources
- [Duckiebot camera calibration](https://docs.duckietown.com/ente/duckietown-manual/20-operations/04-calibrations/duckiebot-camera-calibration.html)
- [Duckiebot wheel calibration](https://docs.duckietown.com/ente/duckietown-manual/20-operations/04-calibrations/duckiebot-motor-odometry-calibration.html)
- [Duckiebot hardware debugging](https://docs.duckietown.com/ente/duckietown-manual/21-testing-debugging/02-components-testing/index.html)

## Implementation Details

### Path Planning
April Tags. Explore until found, do not explore previously explored areas.



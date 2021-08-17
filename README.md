# *sherlock* ROS package

This repository represent the *sherlock* ROS pacakge, the highest package in the hierarchy that represents the robotic mission under experimentation. The *sherlock* package is intended for ROS Melodic. The repository needs to be cloned to the catkin workspace on the robot (no need to clone it to the PC) and compiled with following commands:
```bash
cd <catkin_ws_dir>/src
git clone git@github.com:minana96/sherlock.git
cd ..
catkin_make
```

## Dependencies

The package is dependent on several ROS pacakges, whereas the instruction on how to configure and install those packages are contained in their respective GitHub repositories:
- **sherlock_bringup**: package represents core TurtleBot3-specific ROS nodes and their configuration, instructions available [here](https://github.com/minana96/sherlock_bringup);
- **raspicam_node**: package represents the driver for Raspberry Pi Camera Module v2, instructions available [here](https://github.com/minana96/raspicam_node);
- **ros_melodic_profilers**: package represents profilers for energy consumption, CPU usage and RAM utilisation, instructions available [here](https://github.com/minana96/ros_melodic_profilers);
- **sherlock_slam**: pacakge represents SLAM task and its configuration, instructions available [here](https://github.com/minana96/sherlock_slam);
- **sherlock_navigation**: package represents navigation task and its configuration, instructions available [here](https://github.com/minana96/sherlock_navigation);
- **sherlock_obj_recognition**: package represents object recognition task and its configuration, instructions available [here](https://github.com/minana96/sherlock_obj_recognition);
- **sherlock_localisation**: package represents localisation task and its configuration, instructions available [here](https://github.com/minana96/sherlock_localisation).

## Package content

The package consists of three directories, namely, *config*, *launch* and *src*.

### src

This directory contains source code of two ROS nodes:
- **sherlock_controller.py**: node that orchestrates the robot's movements by sending goal locations to the navigation task;
- **sherlock_obj_recognition_results.py**: node that subscribes to and logs the results of object recognition task.

### config

The direcotry contains parameter configuration for the nodes in *yaml* format that are passed as arguments when the nodes are launched:
- **desired_path.yaml**: the configuration for *sherlock_controller* ROS node. The file defines *reference_frame*, a frame relatively to which are the goal locations expressed (*string*), *time_to_wait* in seconds after the goal location is reached (*integer*), and *goal_poses*, the array of goal poses expressed in (x, y, theta) format relatively to *reference_frame* (*x* and *y* are expressed in meters, whereas *theta* is expressed in degrees). The path is defined accordig to the arena used in the experiment, whereas it can be adjusted for other environments if needed.

### launch

The mission for each of the eight experiments is launched via two launch files, one that initiates the necessary ROS nodes in *start_run* Robot Runner event, and the other representing the mission tasks, which is launched in *launch_mission* event. The two types of launch files are located in *start_up* and *launch_mission* subdirectories, respectfully, and explained below.

#### start_up

There are three launch files that can be launched in *start_run* Robot Runner event:
- **start_up.launch**: launches TurtleBo3 specific nodes (from *sherlock_bringup* ROS package), camera driver (from *raspicam_node* ROS package), the static publisher for frame transformations (from *sherlock_bringup* ROS package), profilers (from *ros_melodic_profilers* ROS package) and the node defined in *sherlock_obj_recognition_results.py* in *src* folder of this package. The reason why TurtleBo3 specific nodes and camera drivers are run before the measurements are started is that it takes a while to calibrate the LiDAR sensor and the camera. In Robot Runner, *start_run* event is configured so that it waits until calibration is done so that the calibration delay do not affect the measurements obtained across different experiment runs. The ROS services provided by profilers are also launched in *start_run* event, as they need to be available when the measurements are started. Finally, the ROS node in *sherlock_obj_recognition_results.py* is run in this event, before the object recognition task is launched in *launch_mission* event, as not to miss any of the subscribed results of object recognition;
- **start_up_high_resolution.launch**: the file intiates the exact same ROS nodes as *start_up.launch*, with the only difference of launching the camera driver with different image resolution configured. This file is launched in the respective higher image resolution treatment in image resolution effect experiment;
- **start_up_high_frame_rate.launch**: the file intiates the exact same ROS nodes as *start_up.launch*, with the only difference of launching the camera driver with different image frame rate configured. This file is launched in therespective higher image frame rate treatment in image frame rate effect experiment. 

#### launch_mission

There are eight launch files that can be launched in *launch_mission* Robot Runner event for each of the eight respective experiments:
- **unkown_map.launch**: launches and SLAM, navigation and object recognition, either locally or remotely (passed as launch file parameters) as part of the *Unknown map* experiment setup;
- **kown_map.launch**: launches and localisation, navigation and object recognition, either locally or remotely (passed as launch file parameters) as part of the *Known map* experiment setup. The premade map of the enviorment, defined in *yaml* format, needs to be also passed as a parameter;
- **test_resolution.launch**: launches SLAM, navigation and object recognition as part of the image resolution effect experiment;
- **test_frame_rate.launch**: launches SLAM, navigation and object recognition as part of the image frame rate effect experiment, where frame rate treatment is passed as parameter;
- **test_particles.launch**: launches SLAM, navigation and object recognition as part of the number of particles effect experiment, where number of particles treatment is passed as parameter;
- **test_temporal_updates.launch**: launches SLAM, navigation and object recognition as part of the temporal updates effect experiment, where temporal updates treatment is passed as parameter;
- **test_velocity_samples.launch**: launches SLAM, navigation and object recognition as part of the number of velocity samples effect experiment, where number of velocity samples treatment is passed as parameter;
- **test_sim_period.launch**: launches SLAM, navigation and object recognition as part of the simulation time effect experiment, where simulation time treatment is passed as parameter.

For all mission launch files, the user needs to configure *remote_pc* parameters in *machine* launch tag to fit the descriptions of the target remote PC where SLAM, localisation, navigation and object recognition should be offloaded. The details about *machine* tag are available [here](http://wiki.ros.org/roslaunch/XML/machine).  

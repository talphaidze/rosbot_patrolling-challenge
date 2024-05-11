### ROSBot Patrolling Challenge

This repository contains files for a ROSBot patrolling challenge implemented using ROS (Robot Operating System). The files are structured within a robot file system to enable the robot to follow a white line, avoid obstacles, and detect objects (fruits). In the event of detecting a 'bad' fruit, as indicated in the classification problem, the robot prints 'bad' in the terminal and sends an SSH signal to a local computer to play a specific sound since the robot does not have sound sensors. The dataset used for object detection training is the 'Freshie Fruits' dataset available on Roboflow.

#### File Structure

The files need to be located in the source directory of the catkin package within the source directory of a workspace. To create a catkin package, you can use the following command:

```
catkin_create_pkg <package_name> [dependencies]
```

#### Included Files

- **followline_avoidobstacles.py**: Contains the implementation for line following and obstacle avoiding behavior.
- **seeimages.py**: Implements object detection behavior. (In order to use the file in this repository you need to first train the YOLOv8 model on the 'Freshie Fruits' dataset and download the weights after training)


#### Usage

1. Clone this repository into your ROS workspace's `src` directory.
2. Navigate to the root of your workspace and build the packages:
   ```
   catkin_make
   ```
   ```
   source devel/setup.bash
   ```
3. Launch the nodes using ROS launch files or individually:
   ```
   roslaunch <package_name> <launch_file>
   ```
   or
   ```
   rosrun <package_name> <python_file>
   ```

#### Notes

- Ensure all dependencies are installed and sourced properly before building the packages.
- Customize the behavior and parameters in the source files to fit your specific requirements.

#### Contributors

- Tamar Alphaidze, Majd Barghoutti

Happy patrolling with ROSBot! 🤖🚀 

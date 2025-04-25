
# UWB-ranging-odom-relative-localization

This repository contains code for UWB ranging, odometry, and relative localization.

## Owner

[Kwoky-SWUST](https://github.com/Kwoky-SWUST)


## Installation Instructions

### Prerequisites

Before you begin, ensure you have the following installed:

*   ROS (Robot Operating System) -  This project likely depends on ROS for robot control and data processing.  Refer to the [ROS installation guide](http://wiki.ros.org/Installation) for your specific operating system.
*   C++ compiler (e.g., g++)
*   CMake -  Used for building the project.
*   Required ROS packages (dependencies):  These will depend on the specific nodes used in the project.  You'll need to identify and install them using `apt-get install ros-<distro>-<package-name>` or similar.  Common dependencies might include:
    *   `roscpp`
    *   `rospy` (if any Python nodes are used)
    *   `sensor_msgs`
    *   `geometry_msgs`
    *   `tf`
    *   Packages related to your specific UWB hardware and odometry source.

### Building the Project

1.  **Clone the repository:**

    ```bash
    git clone https://github.com/Kwoky-SWUST/UWB-ranging-odom-relative-localization.git
    cd UWB-ranging-odom-relative-localization
    ```

2.  **Create a ROS workspace (if you don't have one already):**

    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    catkin_init_workspace
    ```

3.  **Move the repository into the `src` directory of your ROS workspace:**

    ```bash
    mv UWB-ranging-odom-relative-localization .
    ```

4.  **Build the project:**

    ```bash
    cd ..
    catkin_make
    ```

5.  **Source the ROS environment:**

    ```bash
    source devel/setup.bash
    ```

    Add this line to your `.bashrc` or `.zshrc` file to automatically source the environment each time you open a new terminal.

## Key Features

*   **UWB Ranging:**  Implements functionality for reading and processing Ultra-Wideband (UWB) ranging data.
*   **Odometry Integration:**  Integrates odometry data (e.g., from wheel encoders or an IMU) to estimate robot pose.
*   **Relative Localization:**  Combines UWB ranging and odometry to perform relative localization, allowing robots to determine their positions relative to each other or to known anchor points.
*   **Potential for Multi-Robot Systems:** The directory structure suggests support for multi-robot applications.
*   **Sensor Integration:** Includes directories for `hokuyo_node` and `rplidar_ros`, indicating support for laser scanners.


# Trajectory Visualization Repository

**trajectory_visualization** package, tested with ROS Noetic on Ubuntu 20.04

## [Demonstration Video](https://drive.google.com/file/d/1Yzpje0CHUridz5hZzh_00_GLUtXqdiov/view?usp=drive_link)
https://drive.google.com/file/d/1Yzpje0CHUridz5hZzh_00_GLUtXqdiov/view?usp=drive_link

## Build
In your workspace src directory:

    git clone https://github.com/akarsh2906/trajectory_visualizer.git
Go to workspace directory and build:

    cd ..
    catkin_make
For this assignment, I have used the **AR100 robot** for simulation.

<br />
<br />

## Trajectory publisher and saver node
Launch your robot simulation,
Then launch navigation,
Then launch the "trajectory_publisher_saver.launch" using :

    roslaunch trajectory_visualizer trajectory_publisher_saver.launch
Move the robot around, or give goal poses, etc. Then to save the trajectory (time, x, y, yaw) data, use:

    rosservice call /save_trajectory "filename: '/home/user/test.csv' duration: 120.0"
Change the filepath and duration as required.


**Parameters used by this node:**
Parameters file (**trajectory_visualizer.yaml**) is present in the config folder of the package

    namespace: ""

Used to specify namespace to be followed, by default it does not use a namespace

    odom_frame: "odom"

The odom frame id of the odometry topic that will be used to get the robot's location

    odom_topic: "odom"

The odometry topic name

    marker_topic: "trajectory_markers"

The marker array topic name

    marker_interval: 0.1

The minimum time interval at which a new marker should be published.
Lesser the value, more markers published. 

    position_tolerance: 0.02

This is tolerance for the X and Y position values, new markers will be only published for a minimum movement of this value in either X or Y directions.
I've done this so that markers aren't constantly being published even when the robot is stationary.

    yaw_tolerance: 0.02

Same like position tolerance but for orientation.

<br />
<br />

## Trajectory reader and publisher node
Run `roscore`
Open RViz and open the config (**visualize.rviz**) present in the rviz directory of the package.
Then launch:

    roslaunch trajectory_visualizer trajectory_reader_publisher.launch trajectory_file:=/home/user/test.csv

Provide the filepath using the launch argument as shown.
Now the trajectory must be visible in RViz.

**Parameters used by this node**

    trajectory_file: "default.csv"

The trajectory file path.

    odom_frame: "odom"

The odom frame id of the odometry topic that will be used to get the robot's location

    marker_topic: "trajectory_markers"

The marker array topic name

<br />
<br />

## Pseudocode
### Trajectory Collection

    START Trajectory Collection
    
    1. Load Parameters:
       - Read namespace, odometry topic, marker topic, frame names, marker interval, and tolerances.
    
    2. Subscribe to Odometry Topic:
       - Listen for incoming odometry messages.
    
    3. On Receiving Odometry Data:
       - Get current robot position (x, y) and orientation (yaw).
       - If time interval since the last marker >= marker_interval:
           - Compare new position & yaw with the last recorded values.
           - If change in position or yaw is greater than the threshold:
               - Store the new trajectory point (time, x, y, yaw).
               - Update last recorded position & yaw.
               - Publish updated markers.
    
    4. Repeat steps 3 for every incoming odometry message.
    
    END Trajectory Collection


### Trajectory Storage

    START Trajectory Storage
    
    1. Wait for a "save_trajectory" service request.
    2. On receiving a request:
       - Extract the requested filename (where to save the trajectory).
       - Extract the requested duration (how much past data to save).
    
    3. Get the current system time.
    
    4. Initialize an empty list filtered_trajectory.
    
    5. Iterate through stored trajectory points:
       - For each point in the trajectory:
           - Check if its timestamp is within the requested duration:
               - If (point.time >= current_time - duration):
                   - Add the point to filtered_trajectory.
    
    6. If filtered_trajectory is empty:
       - Return a failure response:
           - Message: "No trajectory data available in the specified time range."
       - Exit.
    
    7. Open the CSV file for writing:
       - If file fails to open:
           - Log an error message and exit.
    
    8. Write the CSV header: `"time,x,y,yaw"`.
    
    9. Iterate through filtered_trajectory:
       - For each point:
           - Write `time, x, y, yaw` values to the file.
    
    10. Close the file.
    
    11. Return a success response:
       - Message: "Trajectory saved successfully."
    
    END Trajectory Storage

### Trajectory Visualization

    START Trajectory Visualization
    
    1. Load Parameters:
       - Read trajectory file, odometry frame, and marker topic.
    
    2. Read Trajectory Data from File:
       - Open the CSV file.
       - Read and parse each line to extract (time, x, y, yaw).
    
    3. Transform Coordinates:
       - Convert saved "map" frame coordinates to "odom" frame using TF2.
    
    4. Create Markers for RViz:
       - For each trajectory point:
           - Create an arrow marker.
           - Set position (x, y).
           - Convert yaw to quaternion for orientation.
           - Set color and scale.
    
    5. Publish Marker Array.
    
    END Trajectory Visualization



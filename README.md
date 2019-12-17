# Demo - Dynamic Obstacle Avoidance

This is a description of the Dynamic Obstacle Avoidance demo, proj-lf-vop 2019.

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Requires: Duckiebot in configuration `DB18`

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Requires: Camera calibration completed

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Requires: Duckietown as a single straight lane loop

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Requires: [Joystick control](#rc-control)

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) Requires: At least two Duckiebots

![#c5f015](https://placehold.it/15/c5f015/000000?text=+) Results: A Duckiebot running our enhanced lane-following while avoiding static and dynamic obstacles!

***

## 1. Video of expected results

The final demo should demonstrate the following three functionalities; avoiding static Duckies, avoiding static Duckiebots and avoiding moving Duckiebots. The final result should look like this.

<h4 style="text-align:center;"> Static Obstacle Avoidance - Duckie </h4>

![Static Duckie](/demo_media/vid_02.mp4?raw=true "Overtaking Static Duckie")

<h4 style="text-align:center;"> Static Obstacle Avoidance - Duckiebot*</h4>

![Static Duckiebot](/demo_media/vid_01.mp4?raw=true "Overtaking Static Duckiebot")

<h4 style="text-align:center;"> Dynamic Obstacle Avoidance - Duckiebot* </h4>

![Dynamic Duckiebot](/demo_media/moving_duckie_avoidance.gif?raw=true "Overtaking Moving Duckiebot")

* Reference Duckiebots used as obstacles did not include Duckies on top for identification purposes

## 2. Duckietown Setup Notes

The following assumptions about the Duckietown are made to make this demo work:

* A Duckietown with white and yellow lanes is used
* Required tiles: five straight tiles & two 180 degrees turn tiles arranged as a single straight lane loop
* Good consistent lighting, close the room blinds to avoid excessive natural light
* No other Duckiebots should be visible outside of this Duckietown setup. Cover the lane edges to avoid excess visibility of other Duckiebots, if needed
* No other Duckietown elements are needed

The final Duckietown setup should look like this:

![DT_lane](/demo_media/demo_lane.png?raw=true "Lane Setup")




## 3. Duckiebot setup notes

Duckiebots should be set up using the standard `DB18` configuration. No additional modifications are needed.

## Pre-flight checklist

TMake sure you have completed the following before proceeding:

Check: Duckiebot in configuration `DB18`

Check: Latest Duckietown Shell installed

Check: Latest Docker version installed

Check: Duckiebot is able to run  standard Lane Following

## 4. Demo instructions

### 4.1 Start the Required Containers

In order to run the Dynamic Obstacle Avoidance demo, the following containers are needed:

* dt-car-interface

* dt-duckiebot-interface

* dt-core

If these are not already running, start the containers using the following commands:

#### Duckiebot Interface
`$ dts duckiebot demo --demo_name all_drivers --duckiebot_name ![DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy`

#### Car Interface

`$ dts duckiebot demo --demo_name all --duckiebot_name ![DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy`

#### DT core

The dt-core container will be run with the keyboard control command. This is explained in the next section.

### 4.2 Begin the Dynamic Obstacle Avoidance Pipeline

The next step is to run the Dynamic Obstacle container. You can do this by following these steps. Make sure to replace `[DUCKIEBOT_NAME]` with appropriate vehicle name.

On your local machine (with the latest Duckietown shell installed), run the following commands:

1. Clone the demo repository from github

   `~LAPTOP $ git clone https://github.com/duckietown-ethz/proj-lfvop.git`

2. Go to the cloned folder, and make sure you are on the master branch

   `~LAPTOP $ cd proj-lfvop`

   `~LAPTOP $ git checkout master`

3. Build the docker image onto the Duckiebot

   `~LAPTOP $ dts devel build -f --arch arm32v7 -H [DUCKIEBOT_NAME].local`

4. Run the demo container

   `~LAPTOP $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:master-arm32v7`

5. Start the Keyboard control

   `~LAPTOP $ dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64`

6. Within the keyboard control, press **a** to start the lane following. Press **s** if you need to stop.


By now you should have one Duckiebot running the Dynamic Obstacle Avoidance demo. By default, it detects both static and dynamic obstacles. This can be changed by modifying the launch file. For details on this, go to the advanced user section.

Note: Steps 4 and 5 might take several minutes to complete.*

###### Quick note on Implementation
A quick way to verify that everything is setup properly is to look into the portainer and see the active containers. Below is a picture of the containers running on a Duckiebot successfully implementing the dynamic obstacle avoidance demo.

![containers](/demo_media/demo_01.png?raw=true "containers")
Docker will name new containers with random names, in this case the dynamic obstacle avoidance image is running in a container called hungry_benz.

### 4.3 Running the Demos

#### Running Demo 1 - Static Duckie Avoidance

To run the static Duckie avoidance demo perform the following steps:
1. Set up one Duckiebot per the instructions in Section 4.2.
2. Grab one standard yellow Duckie from your usual Duckie supply
3. Place the Duckie somewhere on the straight lane, at least one tile away from the turning tile.
4. Using the Keyboard control (explained in Section 4.2) begin the lane following procedure.

#### Running Demo 2 - Static Duckiebot Avoidance
To run the static Duckiebot avoidance demo perform the following steps:

1. Set up two Duckiebots per the instructions in Section 4.2.
2. Place the first Duckiebot (DB#1) somewhere in the modified Duckietown. It must be aligned with the lane, and be at least one tile away from the 180 degree turning tile.
3. Place the second Duckiebot (DB#2) on the opposite lane. Make sure both Duckiebots are oriented in the same direction (both clockwise, or both counter-clockwise)
4. Using the Keyboard control (explained in Section 4.2) begin the lane following procedure for the moving Duckiebot.

#### Running Demo 3 - Dynamic Duckiebot Avoidance
The Dynamic Duckiebot Avoidance demo follows the same steps as Demo #2, with the addition of the gain modification to change the vehicle speed. To implement it, follow the steps below:

1. Perform steps 1-4 from demo # 2.
2. Decrease the kinematic gain of the non-moving duckiebot until it barely moves. This is done by running the following commands:
3. Run a terminal inside a Docker container for the non-moving Duckiebot

`~ Laptop $ docker -H DUCKIEBOT_NAME.local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:dynamic_logic-arm32v7 /bin/bash`

4. Update the kinematic gain parameter. The default value is set to 1.0. Depending on the specific Duckiebot calibration, different values could produce the same result. The value used for the demonstration was 0.4.

`~ Docker $ rosparam set /DUCKIEBOT_NAME/kinematics_node/gain VALUE`

5. Using the keyboard control, begin the lane following for the non-moving Duckiebot.


## 5. Advanced User

Here is a list of advanced functionalities that can be performed for this demo. These are not needed to make the project run, but can provide an additional boost to the overall performance.

### Updating the Lane filter parameters
The lane filter was found to work better after trying different segment threshold parameters, as well as matrix mesh sizes. These can be updated using the following commands:

#### Changing Lane Filter Matrix Mesh Size:
1. Run a terminal inside a docker container

`~ Laptop $ docker -H DUCKIEBOT_NAME.local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:dynamic_logic-arm32v7 /bin/bash`

2. Update the parameter for the matrix mesh size

`~Docker $ rosparam set /pato/lane_filter_node/matrix_mesh_size VALUE`

#### Changing max segment threshold:
1. Run a terminal inside a docker container

`~ Laptop $ docker -H DUCKIEBOT_NAME.local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:dynamic_logic-arm32v7 /bin/bash`

2. Update the parameter for the matrix mesh size

`~Docker $ rosparam rosparam set /DUCKIEBOT_NAME/line_detector_node/segment_max_threshold VALUE`

For our demonstration, the following values were used: Matrix mesh size of 0.6 and max segment threshold of 15.  For details on how these were selected, please refer to the official project report.

### Modifying Node Detection Parameters
By default, the Dynamic Obstacle Avoidance node detects Duckiebots and Duckies in the environment. If the types of obstacles are known, the detection can be optimized by limiting the detection only to those.

In the dynamic_obstacle_avoidance.launch launch file, the project nodes are listed. The only required node is the **dynamic_controller_node**. The *led_detection_node* and the *duckie_detection_node* can be commented out if not desired. This section assumes a advanced knowledge of launch files from the user.


## 6. Troubleshooting

**Symptom:** Duckiebot detects Duckie when it's not there

Resolution: Adjust room lighting so that there's no direct reflection onto the Duckiebot's camera.

**Symptom:** Duckiebot is not moving

Resolution: Dynamic Obstacle Avoidance Container might take up to a minute to initialize. If it is taking longer, try to rebuild the container.

**Symptom:** The `docker run` command results in an error

Possible Resolutions:
* Make sure the `[DUCKIEBOT_NAME]` placeholder is replaced with the name of your Duckiebot.
* Make sure to build from the master branch. The container should be called `duckietown/proj-lfvop:master-arm32v7`, if this is not the case, redo step 4.2.

**Symptom:** DTS command returns an error

Resolution: Update to the latest Duckietown Shell and try again.


## 7. Demo Failure Demonstration

Do as we ask, otherwise terrible things might happen!

![failure](/demo_media/demo_03.gif?raw=true "failure_demo")


## 8. Additional Information - Code Structure

# Demo - Dynamic Obstacle Avoidance

This is a description of the Dynamic Obstacle Avoidance demo, proj-lf-vop 2019.

:red_circle: Requires: Duckiebot in configuration `DB18` using  `daffy` version

:red_circle: Requires: Camera calibration completed

:red_circle: Requires: Duckietown as a single straight lane loop

:red_circle: Requires: Joystick control

:red_circle: Requires: At least two Duckiebots

:white_check_mark: Results: A Duckiebot running our enhanced lane-following while avoiding static and dynamic obstacles!

***

## 1. Video of expected results

The demo should demonstrate the following three functionalities; avoiding static Duckies, avoiding static Duckiebots and avoiding moving Duckiebots. The final result should look like this.

<h4 style="text-align:center;"> Static Obstacle Avoidance - Duckie </h4>

![Static Duckie](/demo_media/duckie_overtake.gif?raw=true "Overtaking Static Duckie")

<h4 style="text-align:center;"> Static Obstacle Avoidance - Duckiebot* </h4>

![Static Duckiebot](/demo_media/static_duckiebot.gif?raw=true "Overtaking Static Duckiebot")

<h4 style="text-align:center;"> Dynamic Obstacle Avoidance - Duckiebot* </h4>

![Dynamic Duckiebot](/demo_media/moving_duckie_avoidance.gif?raw=true "Overtaking Moving Duckiebot")

\* Reference Duckiebots used as obstacles did not include Duckies on top for identification purposes

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

Duckiebots should be set up using the standard `DB18` configuration with `daffy` version. No additional modifications are needed.

## Pre-flight checklist

Make sure you have completed the following before proceeding:

Check: Duckiebot in configuration `DB18` using `daffy` version§§§

Check: Latest Duckietown Shell installed

Check: Latest Docker version installed

Check: Duckiebot is able to run  standard Lane Following

## 4. Demo instructions

Make sure to always replace `[DUCKIEBOT_NAME]` with appropriate vehicle name.

### 4.1 Start the Required Containers

In order to run the Dynamic Obstacle Avoidance demo, the following containers are needed:

* dt-car-interface

* dt-duckiebot-interface

* Dynamic Obstacle Avoidance container (called after this repository: proj-lfvop)

If these are not already running, start the containers using the following commands:

#### Duckiebot Interface
`$ dts duckiebot demo --demo_name all_drivers --duckiebot_name [DUCKIEBOT_NAME] --package_name duckiebot_interface --image duckietown/dt-duckiebot-interface:daffy`

#### Car Interface

`$ dts duckiebot demo --demo_name all --duckiebot_name [DUCKIEBOT_NAME] --package_name car_interface --image duckietown/dt-car-interface:daffy`

#### Dynamic Obstacle Avoidance

Our Dynamic Obstacle Avoidance container, which is based on dt-core, will run lane following. The lane following will be actived by using the keyboard control. This is explained in the next section.

### 4.2 Prepare the Dynamic Obstacle Avoidance

The next step is to prepare the Dynamic Obstacle image. You can do this by following these steps.

On your local machine (with the latest Duckietown shell installed), run the following commands:

1. Clone the demo repository from github

   `~LAPTOP $ git clone https://github.com/duckietown-ethz/proj-lfvop.git`

2. Go to the cloned folder, and make sure you are on the master branch

   `~LAPTOP $ cd proj-lfvop`

   `~LAPTOP $ git checkout master`
   
3. Remember to stop the watchtower when building new images
   
   `~LAPTOP $ dts devel watchtower stop -H [DUCKIEBOT_NAME].local`

4. Build the docker image onto the Duckiebot

   `~LAPTOP $ dts devel build -f --arch arm32v7 -H [DUCKIEBOT_NAME].local`

   By now you should have a image called `duckietown/proj-lfvop:master-arm32v7` on your Duckiebot, ready to be run.

   Note: The build might take several minutes to complete. Also running the image as it is done in the demos below will also take some time every time until every thing is ready, so be patient.

### 4.3 Running the Demos

###### Quick note on Implementation
A quick way to verify that everything is setup properly is to look into the portainer and see the active containers (e.g. `http://[DUCKIEBOT_NAME].local:9000/#/containers`). Below is a picture of the containers running on a Duckiebot successfully running the dynamic obstacle avoidance image.

![containers](/demo_media/demo_01.png?raw=true "containers")
Docker will name new containers with random names, in this case the dynamic obstacle avoidance image is running in a container called hungry_benz.

#### Running Demo 1 - Static Duckie Avoidance

To run the static Duckie avoidance demo perform the following steps:

1. Set up one Duckiebot per the instructions in Section 4.2.

2. Run the demo container using the prepared image from step 1.

   `~LAPTOP $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -e STATIC=1 -v /data/:/data/ duckietown/proj-lfvop:master-arm32v7`

   Note: The option `-e STATIC=1` reduces the distance which is droven on the left lane, since the obstacle is not moving. By default the option is turned off.

3. Open a new terminal and start the keyboard control

   `~LAPTOP $ dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64`

4. Grab one standard yellow Duckie from your usual Duckie supply and place the Duckie somewhere on the straight lane, at least one tile away from the turning tile.

5. Use the keyboard control to start the lane following. This is done by pressing `a` within the keyboard control, , press `s` to stop.

The Duckiebot should hopefully safely overtake the Duckie.

#### Running Demo 2 - Static Duckiebot Avoidance
To run the static Duckiebot avoidance demo (one moving Duckiebot (DB#1) overtakes another non-moving Duckiebot (DB#2)) perform the following steps:

1. Set up both Duckiebots per the instructions in Section 4.2.

2. Run the demo container on both Duckiebots using the prepared image from step 1.

   `~LAPTOP $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -e STATIC=1 -v /data/:/data/ duckietown/proj-lfvop:master-arm32v7`

   Note: On DB#2 the container is also run in order to change the colors of the LED's (If the LED's of DB#2 already have the required colors e.g. when Demos 1-3 was run before on it, this step can be skipped.) Once the Duckiebots LED's have changed the color to two reds in the back and two whites in the front (the middle one should be turned off), the container can be killed again on DB#2 by pressing `Ctrl-C` in the according terminal.  

3. Place the first Duckiebot somewhere in the modified Duckietown. It must be aligned with the lane, and be at least two tile away from the 180 degree turning tile.

4. Place the second Duckiebot on the opposite lane. Make sure both Duckiebots are oriented in the same direction (both counter-clockwise).

5. For the overtaking Duckiebot open a new terminal and start the keyboard control

   `~LAPTOP $ dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64`

6. Use the keyboard control to start the lane following. This is done by pressing `a` within the keyboard control, , press `s` to stop.

The overtaking Duckiebot should hopefully safely overtake the standing Duckiebot.

#### Running Demo 3 - Dynamic Duckiebot Avoidance
For the Dynamic Duckiebot Avoidance demo, the overtaking Duckiebot (DB#1) and the Duckiebot (DB#2) which is overtaken are set up differently. Follow the instructions below:

1. Set up two Duckiebots per the instructions in Section 4.2.


Next we will set up the slow moving DB#2. DB#2 will run standard lane following, but the LED's need to be changed to the pattern already described in Demo 2. In addition, DB#2 is made as slow as possible by reducing the gain. In order to do that, follow the steps below (make sure to use the DB#2 name for `[DUCKIEBOT_NAME]`):

2. Run the demo container on DB#2 using the prepared image from step 1 in order to adjust the LED's. If the LED's already have the required colors e.g. when Demos 1-3 was run before with this bot, this step can be skipped.

   `~LAPTOP $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:master-arm32v7`

   As soon as the DB#2's LED's have changed the color to two reds in the back and two whites in the front (the middle one should be turned off), the container can be killed again by pressing `Ctrl-C` in the according terminal.

3. Run the standard lane following demo on DB#2

   `~LAPTOP $ dts duckiebot demo --demo_name lane_following --duckiebot_name [DUCKIEBOT_NAME] --package_name duckietown_demos --image duckietown/dt-core:daffy`

2. Start the keyboard control on DB#2

   `~LAPTOP $ dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64`

5. In a new terminal run the demo container again on DB#2, but this time with the `/bin/bash` addition:

   `~ Laptop $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:dynamic_logic-arm32v7 /bin/bash`

6. Run the following to adjust the gain in the `/bin/bash` just started in the last step

   `~ Docker $ rosparam set /[DUCKIEBOT_NAME]/kinematics_node/gain VALUE`

   Change the gain until DB#2 is barely moving any more (a good starting pint is around 0.4). Use the keyboard control to start/stop the lane following for testing.
   Repeat this step until satisfied. Then this container is not required anymore and can be closed by pressing `Ctrl-C`


By now, you should have the standard lane following be running on DB#2 and a terminal with the keyboard control running.

Next, DB#1 will be set up with our container and increased the gain (make sure to use the DB#1 name for `[DUCKIEBOT_NAME]`):

7. Run the demo container on DB#1 using the prepared image from step 1.

   `~LAPTOP $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:master-arm32v7`

   Note: The option `-e STATIC=1` is now not used because a moving Duckiebot will be overtaken.

8. In a new terminal start the keyboard control for DB#1

   `~LAPTOP $ dts duckiebot keyboard_control [DUCKIEBOT_NAME] --base_image duckietown/dt-core:daffy-amd64`

9. In a new terminal run the demo container again on DB#1, but this time with the `/bin/bash` addition:

   `~ Laptop $ docker -H [DUCKIEBOT_NAME].local run -it --rm --net host -v /data/:/data/ duckietown/proj-lfvop:dynamic_logic-arm32v7 /bin/bash`

10. Run the following to adjust the gain in the `/bin/bash` just started in the last step.

    `~ Docker $ rosparam set /[DUCKIEBOT_NAME]/kinematics_node/gain VALUE`

    Change the gain (default value is 1.0) to make DB#1 slightly faster, but the lane following still needs to be stable. A gain of 1.2-1.3 should still be possible. Use the keyboard control to start/stop the lane following for testing.
    Repeat this step until satisfied. Then this container (and terminal) is not required anymore and can be closed by pressing `Ctrl-C` and then `Ctrl-D`


11. Place both Duckiebots according to steps 3. and 4. in Demo 2.

Now, you are ready to go

12. Use the keyboard control windows of DB#1 and DB#2 to start lane following for both the Duckiebots. This is done by pressing `a` within the keyboard control windows, , press `s` to stop.

The faster DB#1 should overtake DB#2.

## 5. Advanced User

Here is a list of advanced functionalities that can be performed for this demo. These are not needed to make the project run, but can provide an additional boost to the overall performance.

### 5.1 Updating the Lane filter parameters
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

### 5.2 Modifying Node Detection Parameters
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

**Symptom:** While running a Demo 2 or 3, the overtaking Duckiebot (DB#1) is not recognizing the other Duckiebot (DB#2).

Possible Reason: The Duckiebot is not recognized as a oncoming Duckiebot and is not considered an obstacle to overtake. Check light conditions are as ideal as possble (no reflections, not to dark, not so bright).

First check:
  * While not having lane following active but the demo container running, put DB#1 about 80cm behind DB#2 on the same lane and check the terminal in which the demo container is running. Check for the following prints: `[/DUCKIEBOT_NAME/dynamic_controller_node]: headbot detected` or `[/DUCKIEBOT_NAME/dynamic_controller_node]: backbot detected`
  * If the Duckiebots have the same direction you should see `backbot detected`, if not --> case 1
  * If the Duckiebots are are facing each other you should see `headbot detected`, if not --> case 2

Resolution: Start the `duckietown/proj-lfvop:master-arm32v7` container again, using the option -e THRESHOLD=VALUE (default is 235).
  case 1: Use a value above 235, maybe 240 (this will make the code treat more Duckiebots as backbots, dont go above 245)
  case 2: Use a value below 235, maybe 225 (this will make the code treat more Duckiebots as headbots)
  If you changed the value you can check again, if not better try again with an adjusted value.

## 7. Demo Failure Demonstration

Do as we ask, otherwise terrible things might happen!

![failuer](/demo_media/demo_03.gif?raw=true "failuer_demo")


## 8. Additional Information - Code Structure

This repository is based on the dt-core image. It uses the master launch file from the duckietown_demos package to launch the lane following. For the line detection and the lane filter, the packages from CRA2 were used since they showed a better lane following performance. The general structure of the code is shown in the figure below.

<figure>
<img src="/demo_media/code_structure.png" width="400" >
</figure>

The dynamic_obstacle_avoidance package includes three nodes. The led_detection_node runs the duckiebot detection based on their head and back lights. It publishes two messages for head and back bots separately: the state, which is a boolean indicating if a Duckiebot was detected, and an array with position and velocity of the detected Duckiebots. The duckie_detection_node runs the Duckie detection. It publishes a custom message with state of detected duckie (0: no duckie, 1: Duckie on right lane, 2: Duckie on left lane) and location.

The dynamic_obstacle_node is responsible for the logic to control the overtaking maneuver. It subscribes to the car_cmd topic and publishes it again. The car_cmd is unchanged, except if an emergency stop is detected and a zero speed setpoint is published to the car. The dynamic_obstacle_node checks if all conditions are given to start the overtaking maneuver and then controls it. The overtaking maneuver is performed while still running the standard lane following. The changing of the lane is implemented by gradually increasing the lane_controller parameter d_offset. As a future extension, the gain parameter could also be adjusted to drive faster during overtaking. However due to instability of lane following when driving too fast, this lines have been commented out in the current version of the code.

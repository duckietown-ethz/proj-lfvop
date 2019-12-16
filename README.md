# Demo - Dynamic Obstacle Avoidance

This is a description of the Dynamic Obstacle Avoidance demo, proj-lf-vop 2019.

<div class='requirements' markdown="1">

Requires: Duckiebot in configuration `DB18`

Requires: Duckietown as a single straight lane loop

Requires: Camera calibration completed

Requires: [Joystick control](#rc-control)

Requires: At least two Duckiebots

Results: A Duckiebot running our enhanced lane-following while avoiding static and dynamic obstacles!

</div>

## Video of expected results {#demo-template-expected}

The final result should look like this.

<h3> Static Obstacle Avoidance - Duckie</h3>

  <div figure-id="fig:lane_following_vid">
      <figcaption> Duckie avoidance.
      </figcaption>
      <dtvideo src='vimeo:334931570'/>
  </div>

<h3> Static Obstacle Avoidance - Duckiebot </h3>

  <div figure-id="fig:lane_following_vid">
      <figcaption> Duckiebot Avoidance.
      </figcaption>
      <dtvideo src='vimeo:334931570'/>
  </div>

<h3> Dynamic Obstacle Avoidance - Duckiebot </h4>

  <div figure-id="fig:lane_following_vid">
      <figcaption> Moving Duckiebot Avoidance.
      </figcaption>
      <dtvideo src='vimeo:334931570'/>
  </div>


## Duckietown setup notes {#demo-template-duckietown-setup}

Here, describe the assumptions about the Duckietown, including:

* Layout (tiles types)
* Infrastructure (traffic lights, WiFi networks, ...) required
* Weather (lights, ...)

Do not write instructions on how to build the city here, unless you are doing something very particular that is not in the [Duckietown operation manual](+opmanual_duckietown#duckietowns). Here, merely point to them.

## Duckiebot setup notes {#demo-template-duckiebot-setup}

Write here any special setup for the Duckiebot, if needed.

Do not repeat instructions here that are already included in the [Duckiebot operation manual](+opmanual_duckiebot#opmanual_duckiebot).

## Pre-flight checklist {#demo-template-pre-flight}

The pre-flight checklist describes the steps that are sufficient to ensure that the demo will be correct:

Check: operation 1 done

Check: operation 2 done

## Demo instructions {#demo-template-run}

Here, give step by step instructions to reproduce the demo.

Step 1: XXX

Step 2: XXX

Make sure you are specifying where to write each line of code that needs to be executed, and what should the expected outcome be. If there are typical pitfalls / errors you experienced, point to the next section for troubleshooting.

## Troubleshooting {#demo-template-troubleshooting}

Add here any troubleshooting / tips and tricks required, in the form:


Symptom: The Duckiebot flies

Resolution: Unplug the battery and send an email to info@duckietown.org


Symptom: I run `this elegant snippet of code` and get this error: `a nasty line of gibberish`

Resolution: Power cycle until it works.

## Demo failure demonstration {#demo-template-failure}

Finally, put here video of how the demo can fail, when the assumptions are not respected.

You can upload the videos to the [Duckietown Vimeo account](https://vimeo.com/duckietown) and link them here.

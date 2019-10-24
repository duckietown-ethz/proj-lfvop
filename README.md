# proj-lfvop
AMOD 2019 - Dynamic Obstracle Avoidance

## HOW TO RUN
1. dts devel build -f --arch arm32v7 -H daringduck.local

2. In 4 Terminals run the following (if not running already):

docker -H daringduck.local run -it --privileged --rm -v /data:/data --net=host duckietown/dt-duckiebot-interface:daffy

docker -H daringduck.local run --privileged -it --rm --net host duckietown/dt-car-interface:daffy

dts duckiebot keyboard_control daringduck --base_image duckietown/dt-core:daffy-amd64

docker -H daringduck.local run -it --rm --net=host -v /data:/data -e OFFSET=0 --privileged duckietown/proj-lfvop:bumper_avoid-arm32v7

(OFFSET 0: no offset, 1: left lane, 2, middle of the road, default=0)
3. press 'a' in the keyboard terminal to start lane following and 's' to stop.

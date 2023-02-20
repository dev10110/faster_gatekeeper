# GATEKEEPER + FASTER

A simulation repo which adds GATEKEEPER to the FASTER simulation environment.

I've defined true quadrotor dynamics, so that the tracking isnt perfect. 

## installation
```
git clone <this repo>
git submodule init
git submodule update

docker compose build
```

## usage
From inside the docker containers, run
```
roslaunch gatekeeper all_launch.launch
```
to launch the simulation environment and faster's path planning algorithms

and then
```
roslaunch gatekeeper gatekeeper.launch
```
to launch gatekeeper

# CarND-Path-Planning-Project

This project is my submission for the first project from Term 3 of Self-Driving Car Engineer Nanodegree Program.
If you want to find out more about the setup of the project, you can visit the initial instructions [here](instructions.md). 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Code organisations

I have split my code into a few classes. I have represented all cars on the road with the class [Car](src/Car.cpp),
which has a couple of useful methods and most importantly containing `s`, `s_dot`, `s_ddot`, `d`, `d_dot` and `d_ddot`
of the car it represents.

The map data is loaded and held in an instance of class [Map](src/Map.cpp), which can
convert Frenet to Cartesian coordinates as well as calculating speed from Cartesian to Frenet.
The conversion from Frenet to Cartesian utilises [spline.h](src/spline.h) - an open-source library provided by Tino Kluge.

There is a [PredictionModule](src/PredictionModule.cpp), which is solely responsible for generating a list of predictions
for the supplied cars.

The module for generating trajectories is called [TrajectoryGenerator](src/TrajectoryGenerator.cpp) and maintains
the state of the previously generated and submitted to the simulator path in Frenet. It also provides a `JMT` implementation
as well as a couple of methods for generating, updating and refreshing a trajectory.

The [BehaviourPlanner](src/BehaviourPlanner.cpp) module is responsible for generating a plan and choosing the best of
a list of possible trajectories based on their cost. It also contains a few methods which help achieving that goal.

There is a list of [helper functions](src/helpers.h) and constants, which are used in one or more of the above mentioned
classes.

## High-level sequence of operations

Here I will outline the sequence of events in [main.cpp](src/main.cpp). All these actions are performed on instances of 
the earlier mentioned classes. When the simulator makes a call to the running application, a Sense-Plan-Act cycle is started.
The first thing my program does in the cycle is to [update the Ego car's state](src/main.cpp#L75-L84). If this is the
first time the state is to be updated, the coordinates are taking from the simulator. For subsequent calls, the program
relies on the previously submitted trajectory and it moves the car a number of steps along that trajectory. This way we
can get more accurate information in Frenet, since the simulator's Frenet coordinates differ from our Frenet approximation.
Also the simulator moves the car along the provided steps perfectly and we can be sure that the Ego car will be at one
of the previously provided waypoints.

After that the trajectory history gets updated by [removing the waypoints](src/main.cpp#L89) which were already visited,
followed by state update of the other cars on the road [gets updated](src/main.cpp#L94-L106).

The last couple of actions are [generating the predictions](src/main.cpp#L111) for all other cars, using those predictions
and the current state of Ego to [update the plan](src/main.cpp#L112) with the Behaviour planning module and then turning
that plan into the [new trajectory](src/main.cpp#L113) to be submitted to the simulator.
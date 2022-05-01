# Auto
This file handles a lot of different ways to does autonomous related tasks

## Navigation
The navigator provides a global singleton to access and track robot location. 
Has builtin integration with KDrivetrains.
Uses pose estimators to track the robot using relative motion (encoders) and global measurements(vision)

## Trajectory
Builtin class for simple trajectory generation, saving/loading, and hashing

## Playback
Allows for recording of a series of controller inputs and then simulating them on playback

## Pathing
Allows for trajectory generation in the optimal series of points and avoiding obstacles
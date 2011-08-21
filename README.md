# Tennis_ball

Originally designed to track tennis balls through a video sequence, but has been modified to track any colored object. You just need to pass a histogram representing the object to be tracked.

## Command Line

	rosrun tennis_ball tracker

## To Do

* Need to set command line arguments for: histogram, debug
* Need to fix kalman filter so it actually tracks the detected ball
* Need to publish the estimated ball position and velocity
* Need to publish image showing confidence


# drone-stalker
An implementation of a tracker and controller making a Parrot AR.Drone capable of following specified objects.

Useful information
------------------
This is a ROS package, all source code is in Python. The standard AR.Drone keyboard controller is also included.

Tracker: uses openCV to detect specified objects; currently configured to follow a red sphere.

Movement: an implementation of a PID controller to follow the specified object.

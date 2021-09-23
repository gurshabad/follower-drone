An implementation of a tracker and controller making a Parrot AR.Drone capable of following specified objects.

Useful information
------------------
This is a ROS package, all source code is in Python. The standard AR.Drone keyboard controller is also included. You will need to install the quadcopter drivers (apt-get install ros-\*-ardrone-autonomy) before this can work with an AR.Drone.

Tracker: uses openCV to detect specified objects; currently configured to follow a red sphere.

Movement: an implementation of a PID controller to follow the specified object.

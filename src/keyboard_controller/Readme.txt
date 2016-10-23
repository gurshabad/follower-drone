http://stackoverflow.com/questions/6714069/finding-distance-from-camera-to-object-of-known-size
Formula given in the second answer has been used to find the distance of object from camera in the code. You will find the information related to focal length in ardrone_autonomy/data/camera info in the ardrone_autonomy ros package

The code is a rough sketch of the functions used to detect a red coloured object in a frame and find the distance object from the camera given the object size and camera's focal length.
The bagfile is shared with you. Play that bagfile using : rosbag play <bagfile name.bag>

While the file is playing, if you do a rostopic list you should be able to see a topic named /ardrone/image_raw. This is the topic on which the video stream from front camera is coming. 

Make a ros node which subscribes to this topic successfully and in the callback function, detect the red object and the distance of the object from camera using the python code provided for detection. (you would be required to integrate the python code given with your ros node, which should be fairly easy given the code and previous week's experiments involving "keyboard controller" package. You would be making use of the cvbridge package and sensor_msgs/Image datatype for this.

Once you have the ros node to detect the object, implement a kalman filter to track the object. Your code should have proper comments eg: what is your state vector and measurement vector. See how your tracker is performing especially when object goes out of field of view for a few frames. Publish the output of your tracker on a ros topic since this would be subscribed by the controller to be implemented next.

This code will be later integrated with the P, PI and PID controller which you will be implementing next week, hence it is important that this works properly.
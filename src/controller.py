#!/usr/bin/env python

import rospy
from tracking.msg import Posinfo
from geometry_msgs.msg import Twist
'''
PID implementation from: http://code.activestate.com/recipes/577231-discrete-pid-controller/
'''

class PID:
    """
    Discrete PID control
    """

    def __init__(self, P=2.0, I=0.0, D=1.0, Derivator=0, Integrator=0, Integrator_max=500, Integrator_min=-500):

        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min

        self.set_point=0.0
        self.error=0.0

    def update(self,current_value):
        """
        Calculate PID output value for given reference input and feedback
        """

        self.error = self.set_point - current_value

        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error

        self.Integrator = self.Integrator + self.error

        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * self.Ki

        PID = self.P_value + self.I_value + self.D_value

        return PID

    def setPoint(self,set_point):
        """
        Initilize the setpoint of PID
        """
        self.set_point = set_point
        self.Integrator=0
        self.Derivator=0

    def setIntegrator(self, Integrator):
        self.Integrator = Integrator

    def setDerivator(self, Derivator):
        self.Derivator = Derivator

    def setKp(self,P):
        self.Kp=P

    def setKi(self,I):
        self.Ki=I

    def setKd(self,D):
        self.Kd=D

    def getPoint(self):
        return self.set_point

    def getError(self):
        return self.error

    def getIntegrator(self):
        return self.Integrator

    def getDerivator(self):
        return self.Derivator

#Left right PID
lr_pid = PID(P=0.008/6, I=0.00005, D=0)
lr_pid.setPoint(320)

#Up down PID
ud_pid = PID(P=0.008/4, I=0.00005, D=0)
ud_pid.setPoint(175)

#NF Down PID
nf_pid = PID(P=0.002/6, I=0.00005, D=0)
nf_pid.setPoint(300)

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

'''
going near --> positive
going far from balloon --> negative

going left --> positive
going right --> positive

going up --> positive
going down --> negative
'''

i = 1

def callback(data):
    #data.x, data.y, data.distance
    global lr_pid, ud_pid, nf_pid, pub, i
    lr_val = lr_pid.update(data.x)
    ud_val = ud_pid.update(data.y)
    nf_val = nf_pid.update(data.distance)
    msg = Twist()
    nf_val = -1 * nf_val
    if(lr_val > 0.5): lr_val = 0.5
    if(ud_val > 0.5): ud_val = 0.5
    if(nf_val > 0.5): nf_val = 0.5
    if(lr_val < -0.5): lr_val = -0.5
    if(ud_val < -0.5): ud_val = -0.5
    if(nf_val < -0.5): nf_val = -0.5
    rospy.loginfo('I heard %f %f %f', nf_val, lr_val, ud_val)
    msg.linear.x = 0 #nf_val
    msg.linear.y = lr_val
    msg.linear.z = 0 #ud_val
    if (i%6 == 0): pub.publish(msg)
    i = i + 1

def listener():
    rospy.init_node('listener', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)
    rospy.Subscriber('trackerpub', Posinfo, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()



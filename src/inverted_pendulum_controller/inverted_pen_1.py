#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.msg import CurrentState

class InvertedPendulumController:
    def __init__(self):
        rospy.init_node('inverted_pendulum_controller')

        # Control parameters
        self.kp = -2.5
        self.kd = -2.0

        # Set up subscribers and publishers
        self.angle_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, self.angle_callback)
        self.position_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=1)

        # Initialize variables
        self.prev_angle = 3.14

    def angle_callback(self, msg):
        current_angle = msg.curr_theta
        angle_velocity = msg.curr_theta_dot  # Estimate angular velocity using finite differences

        # Compute the control input using the LQR controller
        control_input = self.kp * (current_angle ) + self.kd * angle_velocity

        self.prev_angle = current_angle

        # Publish the control input as the cart's position (force applied to the cart)
        position_msg = ControlForce()
        position_msg.force = control_input
        self.position_pub.publish(position_msg)

if __name__ == "__main__":
    try:
        controller = InvertedPendulumController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

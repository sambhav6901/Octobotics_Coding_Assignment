#!/usr/bin/env python

import rospy
from inverted_pendulum_sim.srv import SetParams  # Replace with the appropriate service type

def call_service():
    # Initialize the ROS node
    rospy.init_node('service_caller_node')

    # Wait for the "set_parameters" service to become available
    rospy.wait_for_service('/inverted_pendulum/set_params')

    try:
        # Create a service proxy to call the "set_parameters" service
        set_parameters = rospy.ServiceProxy('/inverted_pendulum/set_params', SetParams)

        # Prepare the request data
        pendulum_mass = 2
        pendulum_length = 300
        cart_mass = 0.5
        theta_0 = 0
        theta_dot_0 = 0
        theta_dot_dot_0 = 0
        cart_x_0 = 0
        cart_x_dot_0 = 0
        cart_x_dot_dot_0 = 0

        # Call the service and get the result
        result = set_parameters(pendulum_mass, pendulum_length, cart_mass, theta_0, theta_dot_0,theta_dot_dot_0, cart_x_0, cart_x_dot_0, cart_x_dot_dot_0)

        # Process the response
        rospy.loginfo("Position is initialized")

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", str(e))

if __name__ == "__main__":
    call_service()

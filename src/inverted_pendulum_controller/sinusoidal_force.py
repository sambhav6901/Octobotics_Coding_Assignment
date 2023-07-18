#!/usr/bin/env python

import math
import time
import rospy
from inverted_pendulum_sim.msg import ControlForce

import math

def create_sinusoidal_force():
    # Initialize the ROS node
    rospy.init_node('sinusoidal_force_node')

    # Create a publisher to publish the force commands
    force_publisher = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=1)

    # Set the publishing rate (adjust the frequency as needed)
    rate = rospy.Rate(100)  # 10 Hz

    # Amplitude and frequency of the sinusoidal force
    amplitude = 10.0  # N
    frequency = 10.0  # Hz
    sampling_rate = 100  # Number of samples per second (sampling frequency)

    # Time variable for the sinusoidal force
    start_time = time.time()

    while not rospy.is_shutdown():
        # Calculate the force value at the current time
        current_time = time.time() - start_time
        force_value = amplitude * math.sin(2 * math.pi * frequency * current_time)

        # Create a WrenchStamped message to hold the force command
        force_msg = ControlForce()
        force_msg.force = force_value  # Apply the force in the x-direction

        # Publish the force command
        force_publisher.publish(force_msg)

        # Sleep to maintain the publishing rate
        time.sleep(1.0 / sampling_rate)
        rate.sleep()

if __name__ == "__main__":
    create_sinusoidal_force()

#! /usr/bin/env python
import rospy
#from __future__ import print_function

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the ralf action, including the
# goal message and the result message.
import reasoning_and_learning_framework.msg

def ralf_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (RalfAction) to the constructor.
    client = actionlib.SimpleActionClient('ralf :', reasoning_and_learning_framework.msg.RalfAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = reasoning_and_learning_framework.msg.RalfGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A RalfResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('ralf_client_py')
        result = ralf_client()
        print "Result:", ', '.join([str(n) for n in result.sequence])
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

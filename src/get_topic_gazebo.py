#!/usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
import numpy as np
import timeit

def model_states_callback(data):
    """
    Callback function that processes data received from the /gazebo/model_states topic.
    :param data: The received message containing the states of all models.
    """
    j = np.arange(start=0, stop=10, step=1)
    # print(j)
    print(timeit.timeit('"-".join(map(str, range(100)))', number=10000))
    for i, name in enumerate(data.name):
        for jj, indx in enumerate(j):
         # print(jj)
         if (name not in 'small_vertical_tank_clone_%d'%jj):
               if jj>2:
                  continue
         else:
               print(f"Model Name: {name}")
               print(f"Position: {data.pose[i].position}")
            #    print(f"Orientation: {data.pose[i].orientation}")
            #    print(f"Linear Velocity: {data.twist[i].linear}")
            #    print(f"Angular Velocity: {data.twist[i].angular}")
               print("---------")
      #   break

def listenerMS():
    """
    Sets up a ROS node and subscribes to the /gazebo/model_states topic.
    """
    # Initialize the ROS node
    rospy.init_node('model_states_listener', anonymous=True)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    # Keep the script running until the node is shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        listenerMS()
    except rospy.ROSInterruptException:
        pass

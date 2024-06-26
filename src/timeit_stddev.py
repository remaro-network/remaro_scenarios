import random
import timeit
import subprocess
import os
from multiple_objects_collision_detector import MultipleObjectCollisionDetector
import rospy

def average_py(n):
    """Calculate the average of n times execution of program."""
    s = 0
    for i in range(n):
        '''
        class MultipleObjectCollisionDetector:
    def __init__(self, robot_name, object_names, safety_distance, robot_topic):
        self.robot_name = robot_name
        self.robot_position = None
        self.object_names = object_names
        self.robot_index = None
        self.object_indices = []
        self.safety_distance = safety_distance  # a safety distance threshold
        self.collision_distance = 3
        self.object_name = None
        self.robot_topic = robot_topic
        # Subscribe to both the robot and object position topics
        self.model_states_subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.robot_subscriber = rospy.Subscriber(robot_topic, PoseStamped, self.robot_callback)

    def model_states_callback(self, data):
         # Find the indices of the robot and objects in the ModelStates message
        if self.robot_index is None or not self.object_indices:
            for i, name in enumerate(data.name):
                if name == self.robot_name:
                    self.robot_index = i
                elif name in self.object_names:
                    self.object_indices.append(i)

        # Check for each object if it is in collision zone with the robot
        robot_position = data.pose[self.robot_index].position
        
        for index in self.object_indices:
            object_position = data.pose[index].position
            distance = sqrt((robot_position.x - object_position.x) ** 2 +
                            (robot_position.y - object_position.y) ** 2 +
                            (robot_position.z - object_position.z) ** 2)
            
            collision_time = self.robot_position.header.stamp.secs

            if distance < self.safety_distance:
                distance = "%.4f"%(distance)
                rospy.logwarn(f"Collision Warning: {data.name[index]} is within safety threshold of the robot at distance {distance} at {collision_time} seconds.")
            else:
                distance = "%.4f"%(distance)
                rospy.loginfo(f"Distance between robot and {data.name[index]} : {distance} at {collision_time} seconds. No immediate collision risk.")
    
    def robot_callback(self, data):
        self.robot_position = data  
        # self.model_states_callback()

if __name__ == '__main__':
    rospy.init_node('multiple_objects_detector')
    # robot_topic = "/ground_truth_to_tf_bluerov2/pose"  # robot position topic name
    # object_topic = "/gazebo/model_states/"  # object position topic name
    robot_name = "bluerov2"
    object_names = ["small_vertical_tank_clone_0", "large_vertical_tank_clone", \
                    "vertical_tank_quad", "platform", "large_vertical_tank" \
                    "rust_pipe", "oil_drum_clone_0", \
                    "shipping_container", "horizontal_tank_pair", \
                    "bluerov2"]  # Add all the objects you want to monitor
    safety_distance = 5.0  # Safety distance
    robot_topic = "/ground_truth_to_tf_bluerov2/pose"  # robot position topic name
    detector = MultipleObjectCollisionDetector(robot_name, object_names, safety_distance, robot_topic)
    rospy.spin()  # Keep the node running
         '''
    return s / n

n = 10_000_000


curr_dir = os.getcwd()
print(curr_dir)
os.chdir('..')
curr_dir = os.getcwd()
print(curr_dir)

# Use the timeit module to measure execution time of average_py function
timeit_result = timeit.timeit('average_py(n)', globals=globals(), number=1)

print(f"Execution time: {timeit_result} seconds")

# # To calculate standard deviation, you would typically run the timing multiple times and then calculate the standard deviation of those runs.
# # However, directly measuring the standard deviation like %timeit's stdev output isn't straightforward without multiple runs.
# # Here's an example of how you could manually execute multiple runs and calculate the standard deviation.

import numpy as np

def run_timing_test(func_name, n, iterations=1):
    """Run the timing test multiple times and return the times."""
    # Make sure the `func_name` argument is passed as a string representing the function's name.
    # Construct the string to be executed that includes the function call.
    code_to_execute = f'{func_name}({n})'
    # Pass `globals()` to `timeit.timeit()` so it has access to the function and variables.
    times = [timeit.timeit(code_to_execute, globals=globals(), number=1) for _ in range(iterations)]
    return times

# Using the function name as a string when calling the run_timing_test function
times = run_timing_test('average_py', n, 5)

# Calculating the standard deviation of the times
std_dev = np.std(times)

print(f"Execution times: {times}")
print(f"Standard deviation: {std_dev} seconds")
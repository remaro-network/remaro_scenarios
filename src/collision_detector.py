import rospy
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import ModelStates
from math import sqrt

class DistanceCalculator:
    def __init__(self, robot_topic, object_topic):
        self.robot_position = None
        self.object_position_x = None
        self.object_position_y = None
        self.object_position_z = None
        self.safety_distance = 3  # Define a safety distance threshold
        self.object_name = None


        # Subscribe to both the robot and object position topics
        self.robot_subscriber = rospy.Subscriber(robot_topic, PoseStamped, self.robot_callback)
        self.object_subscriber = rospy.Subscriber(object_topic, ModelStates, self.object_callback)

    def robot_callback(self, data):
        self.robot_position = data
        # rospy.loginfo("Time: t={} and Received position: x={}, y={}, z={}".format(data.header.stamp.secs, \
                                                                            #  data.pose.position.x, data.pose.position.y, data.pose.position.z))
   
        self.calculate_and_log_distance()

    def object_callback(self, data):
        for i, name in enumerate(data.name):
            self.object_position_x = data.pose[i].position.x
            self.object_position_y = data.pose[i].position.y
            self.object_position_z = data.pose[i].position.z
            # print(f"Position: {data.pose[i].position}")
            if name == "large_vertical_tank": #small_vertical_tank, bluerov2
                self.object_name = name
                print("Object name: {}".format(name))
                self.calculate_and_log_distance()
                break

    def calculate_and_log_distance(self):
        # Only calculate if both positions have been received
        if self.robot_position is not None and self.object_position_z is not None and self.object_position_y and self.object_position_x:
            distance = sqrt((self.robot_position.pose.position.x - self.object_position_x) ** 2 +
                            (self.robot_position.pose.position.y - self.object_position_y) ** 2 +
                            (self.robot_position.pose.position.z - self.object_position_z) ** 2)
            # Check if the distance is within the collision zone
            if distance < self.safety_distance:
                distance = "%.4f"%(distance)
                rospy.logwarn(f"Collision Warning: Distance ({distance} units) is below safety threshold.")
            else:
                distance = "%.4f"%(distance)
                rospy.loginfo(f"Distance between robot and {self.object_name}: {distance} units. No immediate collision risk.")

if __name__ == '__main__':
    rospy.init_node('distance_calculator_node')
    robot_topic = "/ground_truth_to_tf_bluerov2/pose"  # Replace with your robot position topic name
    object_topic = "/gazebo/model_states/"  # Replace with your object position topic name
    distance_calculator = DistanceCalculator(robot_topic, object_topic)
    rospy.spin()  # Keep the node running


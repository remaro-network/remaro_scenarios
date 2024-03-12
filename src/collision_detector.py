import rospy
from geometry_msgs.msg import Point, PoseStamped
from gazebo_msgs.msg import ModelStates
from math import sqrt

class CollisionDetector:
    def __init__(self, robot_topic, object_topic):
        self.robot_position = None
        self.object_position_x = None
        self.object_position_y = None
        self.object_position_z = None
        self.safety_distance = 10  # Define a safety distance threshold
        self.collision_distance = 3
        self.object_name = None


        # Subscribe to both the robot and object position topics
        self.robot_subscriber = rospy.Subscriber(robot_topic, PoseStamped, self.robot_callback)
        self.object_subscriber = rospy.Subscriber(object_topic, ModelStates, self.object_callback)

    def robot_callback(self, data):
        self.robot_position = data   
        self.calculate_and_log_distance()

    def object_callback(self, data):
        for i, name in enumerate(data.name):
            self.object_position_x = data.pose[i].position.x
            self.object_position_y = data.pose[i].position.y
            self.object_position_z = data.pose[i].position.z
            # print(f"Position: {data.pose[i].position}")
            if name == "large_vertical_tank": #small_vertical_tank_clone_0, bluerov2, large_vertical_tank
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
            collision_time = self.robot_position.header.stamp.secs
            if distance < self.safety_distance and distance > self.collision_distance:
                distance = "%.4f"%(distance)                
                rospy.logwarn(f"Collision Warning: Distance ({distance} meters) is below safety threshold at time {collision_time} seconds.")
            elif distance < self.collision_distance:
                distance = "%.4f"%(distance)
                rospy.logerr(f"Collision Error: Distance is less than ({distance} meters at time {collision_time} seconds.")
            else:
                distance = "%.4f"%(distance)
                rospy.loginfo(f"Distance between robot and {self.object_name}: {distance} meters at time {collision_time} seconds. No immediate collision risk.")

if __name__ == '__main__':
    rospy.init_node('collision_detector')
    robot_topic = "/ground_truth_to_tf_bluerov2/pose"  # robot position topic name
    object_topic = "/gazebo/model_states/"  # object position topic name
    detector = CollisionDetector(robot_topic, object_topic)
    rospy.spin()  # Keep the node running



import rospy
from geometry_msgs.msg import PoseStamped


def position_callback(data):
   # This function will be called every time a new position message is received
   rospy.loginfo("Time: t={} and Received position: x={}, y={}, z={}".format(data.header.stamp.secs, 
                                                                             data.pose.position.x, data.pose.position.y, data.pose.position.z))

def position_sibscriber():
   # Initialize the ROS node
    rospy.init_node('position_sibscriber', anonymous=True)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/ground_truth_to_tf_bluerov2/pose", PoseStamped, position_callback)

    # Keep the script running until the node is shutdown
    rospy.spin()

if __name__ == '__main__':
    try:
        position_sibscriber()
    except rospy.ROSInterruptException:
        pass
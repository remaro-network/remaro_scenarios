import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
import math


collision_free_d_default = 1.0

def calculate_distance_robot_to_obj(x, y, z, xx, yy, zz):

    distance = math.sqrt((xx - x)**2 + (yy - y)**2 + (zz - z)**2)
    if distance < collision_free_d_default:
        print("there is collision here less than {}.".format(collision_free_d_default))
    return distance

def get_robot_position(X, Y, Z):
   print("Bluerov2 positions are X={}, Y = {}, Z = {}".format(X, Y, Z))
   return X, Y, Z

def get_obj_position(xx, yy, zz):
    print("Position x: {} ".format(xx))
    print("Position y: {} ".format(yy))
    print("Position z: {} ".format(zz))
    print("---------")
    return xx, yy, zz


def model_states_callback(data):
    """
    Callback function that processes data received from the /gazebo/model_states topic.
    :param data: The received message containing the states of all models.
    """
    # we have 12 objects in the world, 3 of should not count as objects in the environment
    names_to_check = ["bluerov2", "seafloor", "ocean_surface"] 
    
    for i, name in enumerate(data.name):
        if name not in names_to_check:
            print("Object name:{} ".format(name))

        xx = data.pose[i].position.x
        yy = data.pose[i].position.y
        zz = data.pose[i].position.z
        # print(type(xx))
        get_obj_position(xx, yy, zz)

    
def position_callback(data):

    rospy.loginfo("Time: t={} and Received position: x={}, y={}, z={}".format(data.header.stamp.secs, 
                                                                             data.pose.position.x, data.pose.position.y, data.pose.position.z))
    # print(type(data.pose.position.x))
    get_robot_position(data.pose.position.x, data.pose.position.y, data.pose.position.z)

def listener():
    # Initialize the ROS node
    rospy.init_node('listener', anonymous=True)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)

    # Subscribe to the /gazebo/model_states topic
    rospy.Subscriber("/ground_truth_to_tf_bluerov2/pose", PoseStamped, position_callback)
    
    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
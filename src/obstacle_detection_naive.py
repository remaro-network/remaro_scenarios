import math
import rospy
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates

class ObstacleDetection:
   def __init__(self):
      self.collision_free_d_default = 1.0

      self.robot_position_x = 1
      self.robot_position_y = 1
      self.robot_position_z = 1

      self.obj_position_xx = 4
      self.obj_position_yy = 4
      self.obj_position_zz = 4
      self.obj_name = "small_vertical_tank"
   
   def calculate_distance_robot_to_obj(self):
      distance = math.sqrt((self.obj_position_xx - self.robot_position_x)**2 + 
                         (self.obj_position_yy - self.robot_position_y)**2 +
                          (self.obj_position_zz - self.robot_position_z)**2)
      if distance < self.collision_free_d_default:
         print("there is collision here less than {}.".format(self.collision_free_d_default))
      
      return distance
   
   def get_robot_position(self, X, Y, Z):
      print("Bluerov2 positions are X={}, Y = {}, Z = {}".format(X, Y, Z))
      return X, Y, Z

   def get_obj_position(self, name,  xx, yy, zz):
      print("Name of Obj: {} ".format(name))
      print("Position x: {} ".format(xx))
      print("Position y: {} ".format(yy))
      print("Position z: {} ".format(zz))
      print("---------")
      return xx, yy, zz


   def model_states_callback(self, data):
      """
      Callback function that processes data received from the /gazebo/model_states topic.
      :param data: The received message containing the states of all models.
      """
      # we have 12 objects in the world, 3 of them should not count as objects in the environment
      names_to_check = ["bluerov2", "seafloor", "ocean_surface"] 
      
      for i, name in enumerate(data.name):
         if name not in names_to_check:
               # print("Object name:{} ".format(name))
               self.obj_position_xx = data.pose[i].position.x
               self.obj_position_yy = data.pose[i].position.y
               self.obj_position_zz = data.pose[i].position.z
               self.obj_name = name
               self.get_obj_position(self.obj_name, self.obj_position_xx, self.obj_position_yy, self.obj_position_zz)

    
   def position_callback(self, data):
      self.robot_position_x = data.pose.position.x
      self.robot_position_y = data.pose.position.y
      self.robot_position_z = data.pose.position.z
      self.get_robot_position(self.robot_position_x, self.robot_position_y, self.robot_position_z)

   
   def listener(self):
      # Initialize the ROS node
      rospy.init_node('listener', anonymous=True)

      # Subscribe to the /gazebo/model_states topic
      rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

      # Subscribe to the /gazebo/model_states topic
      rospy.Subscriber("/ground_truth_to_tf_bluerov2/pose", PoseStamped, self.position_callback)
      
      # Keep Python from exiting until this node is stopped
      rospy.spin()
   
if __name__ == '__main__':
   od = ObstacleDetection()
   od.listener()
   distance = od.calculate_distance_robot_to_obj()
   print(distance)







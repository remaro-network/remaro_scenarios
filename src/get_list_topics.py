from numpy import float64
import rosgraph
import rostopic
import rospy

# get rostopic list using python

# not sure if any better or different that master=None
def get_rostopic_list():
   master = rosgraph.Master('/rostopic')
   pubs, subs = rostopic.get_topic_list(master=master)
   topic_data = {}
   print(f"subs {len(subs)}")
   for topic in pubs:
      name = topic[0]
      if name not in topic_data:
         topic_data[name] = {}
         topic_data[name]['type'] = topic[1]
      topic_data[name]['publishers'] = topic[2]
   print(f"subs {len(pubs)}")
   for topic in subs:
      name = topic[0]
      if name not in topic_data:
         topic_data[name] = {}
         topic_data[name]['type'] = topic[1]
      topic_data[name]['subscribers'] = topic[2]

   for topic_name in sorted(topic_data.keys()):
      print(topic_name)

if __name__ == '__main__':
    try:
        get_rostopic_list()
    except rospy.ROSInterruptException:
        pass
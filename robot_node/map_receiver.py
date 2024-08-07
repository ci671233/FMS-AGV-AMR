import rospy
from std_msgs.msg import String
import requests

def receive_map(data):
    map_url = data.data
    response = requests.get(map_url)
    with open('/path/to/save/map.png', 'wb') as f:
        f.write(response.content)
    rospy.loginfo("Map received and saved.")

if __name__ == "__main__":
    rospy.init_node('map_receiver', anonymous=True)
    rospy.Subscriber('map_topic', String, receive_map)
    rospy.spin()

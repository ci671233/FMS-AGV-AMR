import rospy
from std_msgs.msg import String
import json
import base64
import os

def receive_map(data):
    try:
        map_data_base64 = data.data
        map_data = base64.b64decode(map_data_base64)

        os.makedirs(os.path.expanduser('~/map'), exist_ok=True)

        file_path = os.path.expanduser('~/map/map')
        with open(file_path, 'wb') as f:
            f.write(map_data)

        rospy.loginfo(f"Map received and saved to {file_path}.")
    except Exception as e:
        rospy.logerr(f"Error receiving map: {e}")

if __name__ == "__main__":
    rospy.init_node('map_receiver', anonymous=True)
    rospy.Subscriber('/map_topic', String, receive_map)
    rospy.spin()


import rospy
from std_msgs.msg import String
from flask import Flask, request, jsonify
import base64
import os

app = Flask(__name__)

@app.route('/receive_map', methods=['POST'])
def receive_map():
    try:
        data = request.get_json()
        map_data_base64 = data['map_data']
        map_data = base64.b64decode(map_data_base64)

        os.makedirs(os.path.expanduser('~/map'), exist_ok=True)

        file_path = os.path.expanduser('~/map/map')
        with open(file_path, 'wb') as f:
            f.write(map_data)

        rospy.loginfo(f"Map received and saved to {file_path}.")
        return jsonify({"message": "Map received successfully"}), 200
    except Exception as e:
        rospy.logerr(f"Error receiving map: {e}")
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    rospy.init_node('map_receiver', anonymous=True)
    app.run(host='0.0.0.0', port=5000)
    rospy.spin()


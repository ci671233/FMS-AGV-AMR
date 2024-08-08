import rospy
from std_msgs.msg import String
import json
import base64
import os
import subprocess

def receive_map(data):
    try:
        map_info = json.loads(data.data)
        map_data_base64 = map_info['map_data']
        file_extension = map_info['file_extension']  # 파일 확장자를 포함한 부분 추가
        map_data = base64.b64decode(map_data_base64)

        os.makedirs(os.path.expanduser('~/map'), exist_ok=True)

        file_path = os.path.expanduser(f'~/map/map.{file_extension}')
        yaml_path = os.path.expanduser('~/map/map.yaml')

        # 기존 파일 삭제
        for file in os.listdir(os.path.expanduser('~/map')):
            file_path_to_remove = os.path.expanduser(f'~/map/{file}')
            os.remove(file_path_to_remove)

        # 새로운 맵 파일 저장
        with open(file_path, 'wb') as f:
            f.write(map_data)

        rospy.loginfo(f"Map received and saved to {file_path}.")

        # YAML 파일 생성
        with open(yaml_path, 'w') as yaml_file:
            yaml_file.write(f"image: map.{file_extension}\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

        # map_server 노드 실행
        subprocess.Popen(['rosrun', 'map_server', 'map_server', yaml_path])

    except Exception as e:
        rospy.logerr(f"Error receiving map: {e}")

if __name__ == "__main__":
    rospy.init_node('map_receiver', anonymous=True)
    rospy.Subscriber('/map_topic', String, receive_map)
    rospy.spin()

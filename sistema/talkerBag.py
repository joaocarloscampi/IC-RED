import sys
import rospy
import rosbag
import time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped

if __name__ == '__main__':

    if(len(sys.argv) > 1):
        bag = sys.argv[1]
    else:
        print("Insira o nome do arquivo .bag como parâmetro")
        sys.exit()

    bag_in = rosbag.Bag(bag)

    rospy.loginfo("Initializing imu publisher")
    imu_pub = rospy.Publisher('sensor', Imu, queue_size=5)                      # Inicia o objeto que publicará mensagens Imu
    rospy.loginfo("Publishing Imu at: " + imu_pub.resolved_name)

    rospy.loginfo("Initializing vision publisher")
    camera_pub = rospy.Publisher('camera', PoseStamped, queue_size=5)           # Inicia o objeto que publicará mensagens PoseStamped
    rospy.loginfo("Publishing vision at: " + camera_pub.resolved_name)

    rospy.init_node('Simulator', anonymous=True)                                # Inicia o nó do sistema

    rate = rospy.Rate(100)                                                      # 100hz

    for topic, msg, t in bag_in.read_messages(["/sensor", "/camera"]):
        if topic == "/camera":
            rospy.loginfo(msg)
            camera_pub.publish(msg)                                             # Publicando a mensagem da Camera
        if topic == "/sensor":
            rospy.loginfo(msg)
            imu_pub.publish(msg)                                                # Publicando a mensagem Imu
            rate.sleep()

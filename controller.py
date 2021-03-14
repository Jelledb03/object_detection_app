import rospy
from std_msgs.msg import String

def control():
    stop = False
    rospy.init_node('controller_app', anonymous=False)
    image_topic = "/object_detection_node/detections"
    while not stop:
        print("Type something (exit if you want to stop the program): ")
        exit = input()
        if exit.upper() == "EXIT":
            stop = True
        else:
            msg = rospy.wait_for_message(image_topic, String, 5)
            print(msg)



if __name__ == "__main__":
    control()

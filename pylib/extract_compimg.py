import rosbag
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

bag_path = '/data/mid360/bag/cvlabDSR/0529/calibration.bag'  # rosbag path
topic_name = '/cam0/compressed'  # compressed image topic name
save_path = '/data/mid360/bag/cvlabDSR/0529/1.png'  # save image path

# rosbag file open
bag = rosbag.Bag(bag_path, 'r')

# extract msg in topic
for topic, msg, t in bag.read_messages(topics=[topic_name]):
    # msg == compressed image
    # ROS CompressedImage to OpenCV img using CvBridge
    bridge = CvBridge()
    img = bridge.compressed_imgmsg_to_cv2(msg)
    # save image
    cv2.imwrite(save_path, img)
    
    break  # save first image and shutdown

bag.close()  # close rosbag file
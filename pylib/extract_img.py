import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Change the path and topic name
bag_path = '/data/livox/bag/calib03.bag' # rosbag path
topic_name = '/cam0/image_raw' # image topic name
save_path = '/data/livox/multi/image/3.png' # save image path

# rosbag file open
bag = rosbag.Bag(bag_path, 'r')

# extract msg in topic
for topic, msg, t in bag.read_messages(topics=[topic_name]):
    # msg == image
    # ROSimg to OpenCV img using CvBridge
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    # save image
    cv2.imwrite(save_path, img)
        
    break # save first image and shutdown

bag.close() # close rosbag file
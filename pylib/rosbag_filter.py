import rospy
import rosbag
import sys
from rospy.rostime import Time

if len(sys.argv) < 5:
    print("Usage: {} <input.bag> <output.bag> <start_time> <end_time>".format(sys.argv[0]))
    sys.exit(1)

input_bag = sys.argv[1]
output_bag = sys.argv[2]
start_time = rospy.Time.from_sec(float(sys.argv[3]))
end_time = rospy.Time.from_sec(float(sys.argv[4]))

with rosbag.Bag(output_bag, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if start_time <= t <= end_time:
            outbag.write(topic, msg, t)
import rosbag
from std_msgs.msg import Int32  # Import the message type of your topic

bag_file = 'hello1.bag'  # Replace with the path to your bag file
# bag_file = 'waypoint_navigation.bag'
topic_name = '/astrobiolocation'   # Replace with the name of your topic

# Open the bag file
with rosbag.Bag(bag_file, 'r') as bag:
    # Iterate through messages in the specified topic
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        # Process the message (here, we just print it)
        print(f"Timestamp: {t.to_sec()}, Message: {msg}")

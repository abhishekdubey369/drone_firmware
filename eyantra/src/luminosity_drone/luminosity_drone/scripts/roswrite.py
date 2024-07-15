# import rosbag
# from std_msgs.msg import String

# # Specify the bag file name and mode ('w' for write)
# bag_file = 'waypoint_navigation.bag'
# bag = rosbag.Bag(bag_file, 'w')

# # Write messages to the bag file
# for i in range(10):
#     message = String(data=f"Sample message {i}")
#     bag.write('/whycon/poses', message)

# # Close the bag file when done
# bag.close()


import rosbag
from std_msgs.msg import String

def edit_rosbag(input_bag, output_bag, original_type, new_type):
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == '/astrobiolocation' and hasattr(msg, 'whycon_y') and msg.whycon_y == original_type:
                msg.whycon_y = new_type
            outbag.write(topic, msg, t)

if __name__ == '__main__':
    input_bag_file = 'hello.bag'
    output_bag_file = 'hello1.bag'
    original_organism_type = 253.0
    new_organism_type = 400.0

    edit_rosbag(input_bag_file, output_bag_file, original_organism_type, new_organism_type)


import rosbag
import rospy
import os
import argparse

def get_bag_file_path(root_dir, bag_file_name):
    # Construct the full path to the bag file
    bag_file_path = os.path.join(root_dir, bag_file_name)

    return bag_file_path

def get_argparser():
    parser = argparse.ArgumentParser()

    parser.add_argument("--root_rosbag", type=str, 
                    default="/home/sourabh/V2X-Visualization-Framework/ros/catkin_ws/src/Rosbags",

                    choices=["/home/abhidj/catkin_ws/src/cv2x_in2lab_Feb_2023/rosbag/globaldatafusion", 
                             
                             "/home/abhidj/catkin_ws/src/cv2x_in2lab_Feb_2023/rosbag/twizzy",

                             '/home/abhidj/catkin_ws/src/cv2x_in2lab_Feb_2023/rosbag/sdfmast5', "/home/jagtap/Downloads/rosbag_files", "/home/sourabh/V2X-Visualization-Framework/ros/catkin_ws/src/Rosbags"],

                     help='Root directory for mast rosbag files')


    parser.add_argument('--filename', type=str, default = "Drive_test_cpm_working.bag",
                        
                    choices=["mast5_fused_objects_01_feb_2023.bag", 
                             "mast5_fused_objects_15_feb_2023.bag",
                             "Global_data_fusion_6_feb_23.bag",
                            "Twizzy_perception.bag", "Drive_test_cpm_working.bag",
                            "pdk4.bag","filter_pdk.bag"],

                    help='Mast bag file name')
    
    return parser



args = get_argparser().parse_args()

# Get the values of the command-line arguments
mast_root = args.root_rosbag
mast_file = args.filename


#Construct the path required
bag_file_path = get_bag_file_path(mast_root, mast_file)

# Get the topics in the bag



# Open the rosbag file for reading
bag = rosbag.Bag(bag_file_path)

# Iterate over each message in the bag
for topic, msg, t in bag.read_messages():
    print(msg)
    
    # Process the message based on the topic
    
    if topic == "cpm_received":
        # Perform actions specific to the DetectedObjects topic
        # For example, print the message contents
        print("DetectedObjects message:")
    
        

    # Add more if conditions to handle other topics as needed

# Close the rosbag file
bag.close()
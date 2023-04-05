import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import os 
import time
import cv2
from cv_bridge import CvBridge 
import matplotlib.pyplot as plt
import sys

class BagFileParser():
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
        self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
        self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":

        bag_file = "/home/yoda/ros2_ws/src/tracker/tracker/rosbag2_2023_02_22-10_09_42/rosbag2_2023_02_22-10_09_42_0.db3"
        parser = BagFileParser(bag_file)
        
        bridge = CvBridge()

      
        msgs = parser.get_messages("/image_raw") 

        # print(type(msgs[2][1]))
        count = 0 
        for i in msgs:
            img = bridge.imgmsg_to_cv2(i[1])
            filename = os.path.join("stuff", '{0:04d}.jpg'.format(count)) 
            #cv2.imshow("file", img)
            cv2.imwrite(filename, img) 
            count += 1
            if (cv2.waitKey(25) & 0xFF == ord('q')):
              break
            # time.sleep(1)


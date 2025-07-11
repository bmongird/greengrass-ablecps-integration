#!/usr/bin/env python2
import rospy
import json
import socket
import rostopic
from rospy_message_converter import message_converter
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import base64
import os
import time


class ROSListener:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        for i in range (0,3):
            try:
                self.sock.connect(('localhost', 9999))
                break
            except Exception as e:
                if i == 2:
                    print("Failed to connect to AWS bridge. Aborting.")
                    exit(503)
                print("Failed to connect to AWS bridge. Trying again ({}/3)".format(i+1))
                time.sleep(1)
        self.bridge = CvBridge()
        self.frame_counter = 0
    
    def message_callback(self, msg, topic_name):
        try:
            if  topic_name == "/uuv0/camera/image_raw":
                # Skip frames for performance (send every xth frame)
                self.frame_counter += 1
                if self.frame_counter % 15 != 1:
                    return
                
                # Convert ROS image to OpenCV format
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                # self.output_dir = "/app/images"
                # if not os.path.exists(self.output_dir):
                #     os.makedirs(self.output_dir)
                # filename = os.path.join(self.output_dir, "frame_{}.jpg".format("l" if topic_name == "/vu_sss/waterfall_l" else "r"))
                # cv2.imwrite(filename, cv_image)
                
                # Resize to reduce data size 
                small_image = cv2.resize(cv_image, (320, 240))
                
                # Encode as JPEG with lower quality for smaller size
                _, buffer = cv2.imencode('.jpg', small_image, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 40])
                
                # Convert to base64 for JSON transmission
                image_base64 = base64.b64encode(buffer).decode('utf-8')
                
                # Create payload dictionary
                payload_dict = {
                    '_ros_topic': topic_name,
                    'image_data': image_base64,
                    'image_encoding': 'jpeg_base64',
                    'original_width': msg.width,
                    'original_height': msg.height,
                    'compressed_width': 320,
                    'compressed_height': 240,
                    'frame_number': self.frame_counter // 15,  # Actual frames sent
                    'compression_quality': 40
                }

            elif topic_name == "/vu_sss/waterfall_r" or topic_name == "/vu_sss/waterfall_l":
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                _, buffer = cv2.imencode('.jpg', cv_image)                
                image_base64 = base64.b64encode(buffer).decode('utf-8')
                
                payload_dict = {
                    '_ros_topic': topic_name,
                    'image_data': image_base64,
                    'cam_side': 'l' if topic_name == "/vu_sss/waterfall_l" else "r",
                    'image_encoding': 'jpeg_base64',
                    'width': msg.width,
                    'height': msg.height,
                    'something': 'here' #there should be some sort of img_id here 
                }
            
            else:
                # Handle non-image messages normally
                payload_dict = message_converter.convert_ros_message_to_dictionary(msg)
                payload_dict['_ros_topic'] = topic_name
            
            # Send as JSON over socket
            json_data = json.dumps(payload_dict) + '\n'
            self.sock.send(json_data.encode('utf-8'))
            
        except Exception as e:
            rospy.logerr("Failed to send message from %s: %s", topic_name, e)

def main():
    rospy.init_node('ros_listener_bridge', anonymous=True)
    listener = ROSListener()
    rospy.loginfo("ROS listener bridge started")
    
    topics = ['/ground_truth_to_tf_uuv0/pose', '/uuv0/pixhawk_hw', '/uuv0/camera/image_raw', '/uuv0/speed', '/vu_sss/waterfall_r', '/vu_sss/waterfall_l']
    
    for topic in topics:
        ros_topic = rospy.get_param('~ros_topic', topic)
        msg_cls, _, _ = rostopic.get_topic_class(ros_topic, blocking=True)
        
        # Use lambda to pass topic name to callback
        rospy.Subscriber(ros_topic, msg_cls, 
                        lambda msg, topic=ros_topic: listener.message_callback(msg, topic),
                        queue_size=1)
        rospy.loginfo("Listening to %s", ros_topic)
    
    rospy.spin()

if __name__ == '__main__':
    main()
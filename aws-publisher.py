import os
import json
import socket
import threading
from datetime import datetime, timezone
from awsiot.greengrasscoreipc.clientv2 import GreengrassCoreIPCClientV2
from awsiot.greengrasscoreipc.model import JsonMessage, PublishMessage
from awsiot import mqtt_connection_builder

class AWSPublisher:
    def __init__(self):
        self.MQTT_TOPIC = os.getenv("MQTT_TOPIC", "robot")
        self.MQTT_RECEIVE_TOPIC = os.getenv("MQTT_RECEIVE_TOPIC", "iot/uuv0/commands")
        
        self.ipc_client = GreengrassCoreIPCClientV2()
        print("successfully initialized ipc client")

    def on_mqtt_message_received(self, topic, payload, dup, qos, retain, **kwargs):
        print("Received message from topic {}: {}".format(topic, payload), flush=True)
        
    def listen_for_messages(self):
        # ros socket server
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('localhost', 9999))
        server_sock.listen(1)
        
        print("Waiting for ROS bridge connection...")
        conn, addr = server_sock.accept()
        print(f"ROS bridge connected from {addr}")
        
        buffer = ""
        while True:
            try:
                data = conn.recv(65536).decode('utf-8')  
                if not data:
                    break
                    
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            message_dict = json.loads(line)

                            utc_time = datetime.now(timezone.utc)
                            iso = utc_time.isoformat().replace('+00:00', 'Z')

                            message_dict["__typename"] = "GGmsg" 
                            message_dict["createdAt"] = iso
                            message_dict["updatedAt"] = iso
                            if 'header' in message_dict and 'stamp' in message_dict["header"] and 'nsecs' in message_dict["header"]["stamp"]:
                                message_dict["header"]["stamp"]["nsecs"] = int(message_dict["header"]["stamp"]["nsecs"])

                            subtopic = None
                            match message_dict["_ros_topic"]:
                                case "/uuv0/pixhawk_hw":
                                    message_dict["id"] = "xps-pixhawk" 
                                    subtopic = "pixhawk_hw"
                                case "/ground_truth_to_tf_uuv0/pose":
                                    subtopic = "pose"
                                    message_dict["id"] = "xps-pose"
                                case "/uuv0/camera/image_raw":
                                    subtopic = "image"
                                    message_dict["id"] = "xps-image"
                                case "/uuv0/speed": 
                                    subtopic = "speed"
                                    message_dict["id"] = "xps-speed"
                                case '/vu_sss/waterfall_l':
                                    subtopic = "waterfall_l" 
                                    message_dict["id"] = 'waterfall_l'
                                case '/vu_sss/waterfall_r':
                                    subtopic = "waterfall_r"
                                    message_dict["id"] = 'waterfall_r'
                            
                            self.publish_to_gg(message_dict, subtopic)
                            
                        except json.JSONDecodeError as e:
                            print(f"JSON decode error: {e}")
                            print(f"Problematic line length: {len(line)} chars")
                            
            except Exception as e:
                print(f"Socket error: {e}")
                break
                
        conn.close()
        server_sock.close()
    
    def publish_to_gg(self, message_dict, subtopic):
        try:
            payload = json.dumps(message_dict, separators=(',', ':'))
            payload_size = len(payload.encode('utf-8'))
            
            if payload_size > 120000:
                print(f"Warning: Message too large ({payload_size} bytes), skipping...")
                return
            
            full_topic = self.MQTT_TOPIC + '/' + subtopic if subtopic is not None else self.MQTT_TOPIC
            
            message = JsonMessage(message=message_dict)
            self.ipc_client.publish_to_topic(topic=full_topic, publish_message=PublishMessage(json_message=message))
            
        except Exception as e:
            print(f"AWS publish failed: {e}")

def main():        
    publisher = AWSPublisher()
    publisher.listen_for_messages()

if __name__ == '__main__':
    main()
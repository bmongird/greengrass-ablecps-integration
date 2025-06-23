import os
import json
import socket
import threading
from datetime import datetime, timezone
from awscrt import mqtt, http
from awsiot import mqtt_connection_builder

class AWSPublisher:
    def __init__(self):
        # AWS IoT setup
        self.AWS_ENDPOINT = os.getenv("AWS_ENDPOINT")
        self.AWS_PORT = int(os.getenv("AWS_PORT", 8883))
        self.AWS_CERT_FILE = os.getenv("AWS_CERT_FILE", "cert.pem")
        self.AWS_KEY_FILE = os.getenv("AWS_KEY_FILE", "priv.key")
        self.AWS_CA_FILE = os.getenv("AWS_CA_FILE", "AmazonRootCA1.pem")
        self.CLIENT_ID = os.getenv("CLIENT_ID", "ros-mqtt-bridge")
        self.MQTT_TOPIC = os.getenv("MQTT_TOPIC", "robot/pose")
        
        # Proxy support
        PROXY = (os.getenv("HTTPS_PROXY") or os.getenv("https_proxy") or "").replace("http://","").split(":")
        proxy_opts = http.HttpProxyOptions(host_name=PROXY[0], port=int(PROXY[1])) if len(PROXY) == 2 else None
        
        # Create MQTT connection
        self.mqtt_connection = mqtt_connection_builder.mtls_from_path(
            endpoint=self.AWS_ENDPOINT,
            port=self.AWS_PORT,
            cert_filepath=self.AWS_CERT_FILE,
            pri_key_filepath=self.AWS_KEY_FILE,
            ca_filepath=self.AWS_CA_FILE,
            client_id=self.CLIENT_ID,
            clean_session=False,
            keep_alive_secs=30,
            http_proxy_options=proxy_opts
        )
        
        # Connect to AWS IoT
        print(f"Connecting to AWS IoT Core at {self.AWS_ENDPOINT}...")
        self.mqtt_connection.connect().result()
        print("Connected to AWS IoT Core")
        
    def listen_for_messages(self):
        # Create socket server to receive from ROS
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
                # Increased buffer size for image messages
                data = conn.recv(65536).decode('utf-8')  # 64KB chunks instead of 1KB
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
                                    image_data = message_dict.get('image_data', '')
                                    if image_data:
                                        print(f"Image data size: {len(image_data)} characters")
                                        print(f"Estimated payload size: ~{len(json.dumps(message_dict)) / 1024:.1f} KB")
                                case "/uuv0/speed":
                                    subtopic = "speed"
                                    message_dict["id"] = "xps-speed"
                            
                            self.publish_to_aws(message_dict, subtopic)
                            
                        except json.JSONDecodeError as e:
                            print(f"JSON decode error: {e}")
                            print(f"Problematic line length: {len(line)} chars")
                            
            except Exception as e:
                print(f"Socket error: {e}")
                break
                
        conn.close()
        server_sock.close()
    
    def publish_to_aws(self, message_dict, subtopic):
        try:
            # Check message size before publishing
            payload = json.dumps(message_dict, separators=(',', ':'))
            payload_size = len(payload.encode('utf-8'))
            
            if payload_size > 120000:  # 120KB safety margin (AWS limit is 128KB)
                print(f"Warning: Message too large ({payload_size} bytes), skipping...")
                
                # Option: Send metadata only for oversized images
                if subtopic == "image" and 'image_data' in message_dict:
                    metadata_only = {
                        "id": message_dict["id"],
                        "_ros_topic": message_dict["_ros_topic"],
                        "__typename": message_dict["__typename"],
                        "createdAt": message_dict["createdAt"],
                        "updatedAt": message_dict["updatedAt"],
                        "frame_number": message_dict.get("frame_number"),
                        "compressed_width": message_dict.get("compressed_width"),
                        "compressed_height": message_dict.get("compressed_height"),
                        "original_width": message_dict.get("original_width"),
                        "original_height": message_dict.get("original_height"),
                        "status": "image_too_large_for_mqtt"
                    }
                    payload = json.dumps(metadata_only, separators=(',', ':'))
                    print("Sending image metadata only due to size limit")
                else:
                    return
            
            self.mqtt_connection.publish(
                topic=(self.MQTT_TOPIC + "/" + subtopic if subtopic is not None else self.MQTT_TOPIC),
                payload=payload,
                qos=mqtt.QoS.AT_LEAST_ONCE
            )
            
            topic_name = self.MQTT_TOPIC + '/' + subtopic if subtopic is not None else self.MQTT_TOPIC
            print(f"Published message to {topic_name}")
            
        except Exception as e:
            print(f"AWS publish failed: {e}")

def main():
    if not os.getenv("AWS_ENDPOINT"):
        print("Error: Set AWS_ENDPOINT & certificate env-vars first.")
        return
        
    publisher = AWSPublisher()
    publisher.listen_for_messages()

if __name__ == '__main__':
    main()
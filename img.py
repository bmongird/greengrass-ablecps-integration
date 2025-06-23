#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import hashlib
import os

class HeadlessImageAnalyzer:
    def __init__(self):
        rospy.init_node('headless_image_analyzer', anonymous=True)
        
        self.bridge = CvBridge()
        
        # Statistics
        self.frame_count = 0
        self.start_time = time.time()
        self.last_report_time = 0
        self.last_image_hash = None
        self.duplicate_count = 0
        self.last_ros_timestamp = None
        self.last_cv_image = None
        
        # Create output directory for sample images
        self.output_dir = "/app/images"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Subscribe to image topic with queue_size=1 for real-time
        self.image_sub = rospy.Subscriber(
            "/uuv0/camera/image_raw", 
            Image, 
            self.image_callback,
            queue_size=1,
            buff_size=2**24  # 16MB buffer
        )
        
        print("=" * 60)
        print("Headless ROS Image Analyzer Started")
        print("Subscribed to: /uuv0/camera/image_raw")
        print("Output directory: {}".format(self.output_dir))
        print("Will save sample images and report statistics")
        print("=" * 60)
    
    def image_callback(self, msg):
        try:
            current_time = time.time()
            self.frame_count += 1
            
            # Get ROS timestamp
            ros_timestamp = msg.header.stamp.secs + (msg.header.stamp.nsecs / 1e9)
            message_age = current_time - ros_timestamp if ros_timestamp > 0 else 0
            
            # Check for duplicate timestamps
            timestamp_duplicate = False
            timestamp_backwards = False
            timestamp_diff = 0
            
            if self.last_ros_timestamp is not None:
                timestamp_diff = ros_timestamp - self.last_ros_timestamp
                if timestamp_diff == 0:
                    timestamp_duplicate = True
                elif timestamp_diff < 0:
                    timestamp_backwards = True
            
            # Convert ROS image to OpenCV
            try:
                if msg.encoding == 'bgr8':
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                elif msg.encoding == 'rgb8':
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                elif msg.encoding in ['mono8', 'mono16']:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                    if len(cv_image.shape) == 2:
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                else:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception as e:
                print("Image conversion failed: {}".format(e))
                return
            
            # Check for visual duplicates
            image_hash = hashlib.md5(cv_image.tobytes()).hexdigest()[:12]
            visual_duplicate = False
            pixel_diff_percentage = 0
            
            if self.last_image_hash is not None:
                if image_hash == self.last_image_hash:
                    self.duplicate_count += 1
                    visual_duplicate = True
            
            # Calculate pixel differences if we have a previous frame
            if self.last_cv_image is not None:
                try:
                    if self.last_cv_image.shape == cv_image.shape:
                        diff = cv2.absdiff(cv_image, self.last_cv_image)
                        diff_sum = np.sum(diff)
                        pixel_diff_percentage = (diff_sum / (cv_image.size * 255)) * 100
                except Exception as e:
                    print("Pixel diff calculation failed: {}".format(e))
            
            # Save sample images (every 30 frames)
            if self.frame_count % 30 == 1:
                filename = os.path.join(self.output_dir, "frame_{:06d}.jpg".format(self.frame_count))
                cv2.imwrite(filename, cv_image)
                print("Saved sample image: {}".format(filename))
            
            # Report statistics every 5 seconds
            if current_time - self.last_report_time > 5.0:
                fps = self.frame_count / (current_time - self.start_time) if current_time > self.start_time else 0
                
                print("\n" + "=" * 60)
                print("FRAME ANALYSIS REPORT - Frame {}".format(self.frame_count))
                print("=" * 60)
                print("Performance:")
                print("  FPS: {:.2f}".format(fps))
                print("  Total frames: {}".format(self.frame_count))
                print("  Visual duplicates: {}".format(self.duplicate_count))
                print("  Duplicate rate: {:.1f}%".format((self.duplicate_count / float(self.frame_count)) * 100))
                
                print("\nCurrent Frame:")
                print("  Size: {}x{}".format(msg.width, msg.height))
                print("  Encoding: {}".format(msg.encoding))
                print("  ROS timestamp: {:.6f}".format(ros_timestamp))
                print("  Message age: {:.3f}s".format(message_age))
                print("  Image hash: {}".format(image_hash))
                
                if self.last_ros_timestamp is not None:
                    print("  Time since last: {:.6f}s".format(timestamp_diff))
                
                print("\nFrame Status:")
                if timestamp_duplicate:
                    print("  [WARNING] DUPLICATE TIMESTAMP detected!")
                elif timestamp_backwards:
                    print("  [WARNING] BACKWARDS TIMESTAMP detected!")
                else:
                    print("  [OK] Timestamp is new")
                
                if visual_duplicate:
                    print("  [WARNING] VISUAL DUPLICATE detected!")
                else:
                    print("  [OK] Visual content is new")
                
                if pixel_diff_percentage > 0:
                    print("  Pixel difference: {:.2f}%".format(pixel_diff_percentage))
                    if pixel_diff_percentage < 0.1:
                        print("  [INFO] Nearly identical to previous frame")
                
                if message_age > 1.0:
                    print("  [WARNING] Message is old ({:.2f}s), possible backlog!".format(message_age))
                elif message_age > 0.5:
                    print("  [CAUTION] Message delay ({:.2f}s)".format(message_age))
                else:
                    print("  [OK] Message is recent")
                
                print("=" * 60)
                self.last_report_time = current_time
            
            # Update tracking variables
            self.last_image_hash = image_hash
            self.last_ros_timestamp = ros_timestamp
            self.last_cv_image = cv_image.copy()
            
        except Exception as e:
            print("Error processing image: {}".format(e))
    
    def run(self):
        try:
            print("Starting analysis... Press Ctrl+C to stop")
            rospy.spin()
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            # Final report
            total_time = time.time() - self.start_time
            avg_fps = self.frame_count / total_time if total_time > 0 else 0
            
            print("\n" + "=" * 60)
            print("FINAL ANALYSIS REPORT")
            print("=" * 60)
            print("Total runtime: {:.1f} seconds".format(total_time))
            print("Total frames processed: {}".format(self.frame_count))
            print("Average FPS: {:.2f}".format(avg_fps))
            print("Visual duplicates: {}".format(self.duplicate_count))
            print("Duplicate rate: {:.1f}%".format((self.duplicate_count / float(self.frame_count)) * 100 if self.frame_count > 0 else 0))
            print("Sample images saved to: {}".format(self.output_dir))
            print("=" * 60)

def main():
    try:
        analyzer = HeadlessImageAnalyzer()
        analyzer.run()
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print("Error starting analyzer: {}".format(e))

if __name__ == '__main__':
    main()
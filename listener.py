#!/usr/bin/env python2
import rospy
import rostopic
import time

def main():
    rospy.init_node('ros_topic_lister', anonymous=True)
    rospy.loginfo("Getting all available ROS topics...")
    time.sleep(5)

    try:
        # Get all published topics
        topics = rospy.get_published_topics()
        
        print("\nAvailable ROS Topics:")
        print("=" * 50)
        
        for topic_name, topic_type in topics:
            print("Topic: {} | Type: {}".format(topic_name, topic_type))
        
        print("=" * 50)
        print("Total topics found: {}".format(len(topics)))
        
    except Exception as e:
        rospy.logerr("Failed to get topics: %s", e)

if __name__ == '__main__':
    main()
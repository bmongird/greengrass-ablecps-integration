import sys
import time
import json
import traceback

from awsiot.greengrasscoreipc.clientv2 import GreengrassCoreIPCClientV2
from awsiot.greengrasscoreipc.model import (
    SubscriptionResponseMessage,
    UnauthorizedError
)


IPC_CLIENT = GreengrassCoreIPCClientV2()

def main():
    topic = '#'

    try:
        _, operation = IPC_CLIENT.subscribe_to_topic(topic=topic, on_stream_event=on_stream_event,
                                                     on_stream_error=on_stream_error, on_stream_closed=on_stream_closed)
        print('Successfully subscribed to topic: ' + topic)

        try:
            while True:
                time.sleep(10)
        except InterruptedError:
            print('Subscribe interrupted.')

        operation.close()
    except UnauthorizedError:
        print('Unauthorized error while subscribing to topic: ' +
              topic, file=sys.stderr)
        traceback.print_exc()
        exit(1)
    except Exception:
        print('Exception occurred', file=sys.stderr)
        traceback.print_exc()
        exit(1)

def checkPoseMessage(message) -> bool:
    if message['header']['stamp']['nsecs'] == 0:
        return True
    return False

def checkBatteryMessage(message) -> bool:
    if message['header']['stamp']['nsecs'] == 0:
        return True
    return False

def checkSonarMessage(message) -> bool:
    return True

def checkImageMessage(message) -> bool:
    return True

def checkMessage(message, topic) -> bool:
    try:
        topic_parts = topic.split('/')
        if not topic_parts:
            print(f"Invalid topic format: {topic}")
            return False
        topic_type = topic_parts[-1]

        result = False
        match topic_type:
            case "pose":
                result = checkPoseMessage(message)
            case "pixhawk_hw": #battery
                result = checkBatteryMessage(message)
            case "waterfall_l":
                result = checkSonarMessage(message)
            case "waterfall_r":
                result = checkSonarMessage(message)
            case "image":
                result = checkImageMessage(message)
        
        return result
    except:
        print("Failed to extract topic %s" % topic)
        return False

def on_stream_event(event: SubscriptionResponseMessage) -> None:
    try:
        topic = event.json_message.context.topic
        message = event.json_message.message
        if checkMessage(message, topic):
            result = IPC_CLIENT.publish_to_iot_core(topic_name=f"{topic}", qos='0', payload=json.dumps(message)) #
    except:
        traceback.print_exc()


def on_stream_error(error: Exception) -> bool:
    print('Received a stream error.', file=sys.stderr)
    traceback.print_exc()
    return False


def on_stream_closed() -> None:
    print('Subscribe to topic stream closed.')


if __name__ == '__main__':
    main()

#!/usr/bin/env bash
set -e

# Source ROS environment (Python 2.7)
source /opt/ros/melodic/setup.bash        # sets ROS_PACKAGE_PATH, PYTHONPATH, …
cd msgs/bluerov
source devel/setup.bash

# Ask once, or honor an env-var for non-interactive runs
if [[ -z "$NETWORK_SUFFIX" ]]; then
  read -rp "Enter the random network suffix (123 → network_123): " NETWORK_SUFFIX
fi

NET="network_${NETWORK_SUFFIX}"
MASTER="ros-master_${NETWORK_SUFFIX}"
SELF="$(hostname)"                # container name/ID

echo "Connecting this container ($SELF) to Docker network '$NET' …"
docker network connect "$NET" "$SELF"              # needs /var/run/docker.sock
echo "Connected."

# Set ROS env vars
export ROS_MASTER_URI="http://${MASTER}:11311"
export ROS_HOSTNAME="$SELF"

echo "Listening to all topics on $ROS_MASTER_URI …"

# Set AWS IoT environment variables
export AWS_ENDPOINT="a3sa7caljdtq41-ats.iot.us-east-2.amazonaws.com"
export AWS_CERT_FILE="/ben-dell-xps.cert.pem"
export AWS_KEY_FILE="/ben-dell-xps.private.key"
export AWS_CA_FILE="/root-CA.crt"
export CLIENT_ID="basicPubSub"
export MQTT_TOPIC="iot/bluerov"
export ROS_TOPIC="${ROS_TOPIC:-/ground_truth_to_tf_uuv0/pose}"

echo "\nRunning ROS to AWS IoT bridge...\n"

# Start AWS publisher in background (Python 3.10)
echo "Starting AWS IoT publisher (Python 3.10)..."
/usr/local/bin/python3.10 /aws-publisher.py &
AWS_PID=$!

# Give AWS publisher time to start and create socket server
sleep 3

# Start ROS listener (Python 2.7) 
echo "Starting ROS listener (Python 2.7)..."
python2 /ros-listener.py &
ROS_PID=$!

# Function to handle cleanup on exit
cleanup() {
    echo "Shutting down bridge processes..."
    kill $AWS_PID $ROS_PID 2>/dev/null || true
    wait $AWS_PID $ROS_PID 2>/dev/null || true
    echo "Bridge shutdown complete."
}

# Set up signal handlers
trap cleanup SIGTERM SIGINT

echo "Bridge is running. ROS listener PID: $ROS_PID, AWS publisher PID: $AWS_PID"
echo "Press Ctrl+C to stop..."

# Wait for both processes
wait $AWS_PID $ROS_PID
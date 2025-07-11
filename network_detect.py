#!/usr/bin/env python
import docker, datetime, json
import os, subprocess, sys

client = docker.from_env()
filters = {
    "type":   ["network", "container"],
    "event":  ["create", "die", "stop"]
}
containers = {}

def start_listener(name):
    print('Starting listener')
    child_env = os.environ.copy()                 # inherit everything
    child_env['TARGET_TOPIC'] = '/uuv0/fdr_pos_est'
    child_env['NETWORK_SUFFIX'] = name[8:]
    container = subprocess.check_output(['docker-compose', 'run', '-d', '--rm', '--name', 'listener-{0}'.format(name[8:]), '-e', 'NETWORK_SUFFIX={0}'.format(name[8:]), 'listener'], env=child_env).decode().strip()
    print("Listener container started with Docker name {0}".format(container))
    containers[name[8:]] = container
    

print("Initiating new docker network detection")

for ev in client.events(filters=filters, decode=True):
    if ev["Type"] == "network" and ev["Action"] == "create":
        ts   = datetime.datetime.now().strftime("%F %T")
        name = ev["Actor"]["Attributes"].get("name", "")
        nid  = ev["Actor"]["ID"]
        print("{0}  network {1} ({2}) created".format(ts, name, nid))
        if name[:8] == "network_":
            print('Detected ROS simulation network with ID {0}.'.format(name[8:]))
            start_listener(name)
    elif ev["Type"] == "container" and (ev["Action"] == "die" or ev["Action"] == "stop"):
        name = ev["Actor"]["Attributes"].get("name", "")
        for key in containers:
            if name == "ros-master_{0}".format(key):
                print(client.containers.list())
                client.containers.get(containers[key]).remove(force=True)
                print("[listener] removed {0}".format(containers.pop(key)))
                break


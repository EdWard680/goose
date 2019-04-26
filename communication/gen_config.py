#!/usr/bin/env python3

import yaml
import os

HOST = "192.168.1.70"
NUM_DUCKS = 3

RSSI_MSG_TYPE = "std_msgs.msg:Int8"
DIST_MSG_TYPE = "std_msgs.msg:Float64"
ODOM_MSG_TYPE = "nav_msgs.msg:Odometry"

DIST_PARAMS = ["filtered_distance", "raw_distance"]

def print_list_of_dicts(l):
    for d in l:
        for k,v in d.items():
            print(k,':',v)
        print()

def gen_ros_to_mqtt(this_duck,other_ducks):
    rssi = [
                {
                    "factory":"mqtt_bridge.bridge:RosToMqttBridge",
                    "msg_type": RSSI_MSG_TYPE,
                    "topic_from":"duck{}/rssi/duck{}".format(this_duck,other_duck),
                    "topic_to":"duck{}/rssi/duck{}".format(this_duck,other_duck)
                }
                for other_duck in other_ducks
            ]

    dist = [
                {
                    "factory":"mqtt_bridge.bridge:RosToMqttBridge",
                    "msg_type": DIST_MSG_TYPE,
                    "topic_from":"duck{}/{}/duck{}".format(this_duck,param,other_duck),
                    "topic_to":"duck{}/{}/duck{}".format(this_duck,param,other_duck)
                }
                for param in DIST_PARAMS for other_duck in other_ducks
            ]

    odom = [
                {
                    "factory":"mqtt_bridge.bridge:RosToMqttBridge",
                    "msg_type": DIST_MSG_TYPE,
                    "topic_from":"duck{}/odometry".format(this_duck),
                    "topic_to":"duck{}/odometry".format(this_duck)
                }
            ]
    l = [*rssi,*dist,*odom]
    # print_list_of_dicts(l)
    return l

def gen_mqtt_to_ros(this_duck,other_ducks):
    ducks = [this_duck, *other_ducks]
    rssi = [
                {
                    "factory":"mqtt_bridge.bridge:MqttToRosBridge",
                    "msg_type": RSSI_MSG_TYPE,
                    "topic_from":"duck{}/rssi/duck{}".format(other_duck,duck),
                    "topic_to":"duck{}/rssi/duck{}".format(other_duck,duck)
                }
                for other_duck in other_ducks for duck in ducks if duck is not other_duck
            ]

    dist = [
                {
                    "factory":"mqtt_bridge.bridge:MqttToRosBridge",
                    "msg_type": DIST_MSG_TYPE,
                    "topic_from":"duck{}/{}/duck{}".format(other_duck,param,duck),
                    "topic_to":"duck{}/{}/duck{}".format(other_duck,param,duck)
                }
                for param in DIST_PARAMS for other_duck in other_ducks for duck in ducks if duck is not other_duck
            ]

    odom = [
                {
                    "factory":"mqtt_bridge.bridge:MqttToRosBridge",
                    "msg_type": DIST_MSG_TYPE,
                    "topic_from":"duck{}/odometry/".format(other_duck),
                    "topic_to":"duck{}/odometry/".format(other_duck)
                }
                for other_duck in other_ducks
            ]
    l = [*rssi,*dist,*odom]
    # print_list_of_dicts(l)
    return l

def gen_config_dict(this_duck, other_ducks):
    a = gen_ros_to_mqtt(this_duck, other_ducks)
    b = gen_mqtt_to_ros(this_duck, other_ducks)
    config = {
                'mqtt': {
                    'connection': 
                    {
                        'host': 'localhost', 'port': 1883, 'keepalive': 60
                    }, 
                    'client': {
                        'protocol': 4
                    }
                }, 
                'bridge': [*a, *b]
            }
    return config

if __name__ == '__main__':
    
    this_duck = os.environ.get("ID") or -1
    other_ducks = [duck for duck in range(NUM_DUCKS) if duck is not this_duck]
    config = gen_config_dict(this_duck,other_ducks)

    with open("config.yaml", 'w') as f:
        yaml.dump(config, f)

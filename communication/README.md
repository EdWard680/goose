# ROS MQTT Bridge
This node handles ROS <-> MQTT commmunication using the [mqtt_nridge](https://github.com/groove-x/mqtt_bridge) and the resources from the [mqtt_bridge portion of the Docker-ROS repo](https://github.com/DTU-R3/Docker-ROS/tree/master/r3-mqtt-bridge)

If the mqtt broker is not started, start it using:
`docker -H 192.168.1.70 run -dit -p 1883:1883 -p 9001:9001 eclipse-mosquitto`

# Topics
If you have Duck I, Duck J, and Duck K, and the the bridge is running on Duck I:
## ROS to MQTT

### Distances
```
duck<I>/filtered_distance/duck<M> -> duck<I>/filtered_distance/duck<J>
duck<I>/filtered_distance/duck<M> -> duck<I>/filtered_distance/duck<K>
duck<I>/raw_distance/duck<J> -> duck<I>/raw_distance/duck<J>
duck<I>/raw_distance/duck<J> -> duck<I>/raw_distance/duck<K>
```
These are the distances measured by duck I to duck J

### Odometry
```
duck<I>/odometry/x -> duck<I>/odometry/x
duck<I>/odometry/y -> duck<I>/odometry/y
duck<I>/odometry/pose -> duck<I>/odometry/pose
```
These are duck I's odometry measurements

## JQTT to ROS

### Distances
```
duck<J>/filtered_distance/duck<I> -> duck<J>/filtered_distance/duck<I>
duck<J>/filtered_distance/duck<K> -> duck<J>/filtered_distance/duck<K>
duck<K>/filtered_distance/duck<I> -> duck<J>/filtered_distance/duck<I>
duck<K>/filtered_distance/duck<J> -> duck<J>/filtered_distance/duck<J>
duck<J>/raw_distance/duck<I> -> duck<J>/raw_distance/duck<I>
duck<J>/raw_distance/duck<K> -> duck<J>/raw_distance/duck<K>
duck<K>/raw_distance/duck<I> -> duck<J>/raw_distance/duck<I>
duck<K>/raw_distance/duck<J> -> duck<J>/raw_distance/duck<J>
```
Distances recieved over JQTT measured by duck J/K to each other duck

### Odometry
```
duck<J>/odometry/x -> duck<J>/odometry/x
duck<J>/odometry/y -> duck<J>/odometry/y
duck<J>/odometry/pose -> duck<J>/odometry/pose
duck<K>/odometry/x -> duck<K>/odometry/x
duck<K>/odometry/y -> duck<K>/odometry/y
duck<K>/odometry/pose -> duck<K>/odometry/pose
```
Odometry measurements from each other duck
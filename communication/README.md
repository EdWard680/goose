# ROS MQTT Bridge
This node handles ROS <-> MQTT commmunication using the [mqtt_nridge](https://github.com/groove-x/mqtt_bridge) and the resources from the [mqtt_bridge portion of the Docker-ROS repo](https://github.com/DTU-R3/Docker-ROS/tree/master/r3-mqtt-bridge)

If the mqtt broker is not started, start it using:
`docker -H 192.168.1.70 run -dit -p 1883:1883 -p 9001:9001 eclipse-mosquitto`

# Topics
If you have Duck I, Duck J, and Duck K, and the the bridge is running on Duck I:
## ROS to MQTT
`ros_topic -> mqtt_topic`
### RSSI
```
rssi/duck<I>_duck<J> -> rssi/duck<I>_duck<J>
rssi/duck<I>_duck<K> -> rssi/duck<I>_duck<K>
```
I is less than J and K
<!-- ### Distances
```
duck<I>/filtered_distance/duck<M> -> duck<I>/filtered_distance/duck<J>
duck<I>/filtered_distance/duck<M> -> duck<I>/filtered_distance/duck<K>
duck<I>/raw_distance/duck<J> -> duck<I>/raw_distance/duck<J>
duck<I>/raw_distance/duck<J> -> duck<I>/raw_distance/duck<K>
```
These are the distances measured by duck I to duck J -->

### Odometry
```
odometry/duck<I> -> odometry/duck<I>
```
Duck I's odometry measurements

## MQTT to ROS
`mqtt_topic -> ros_topic`
### RSSI
```
rssi/duck<J>_duck<K> -> rssi/duck<J>_duck<K>
```
J is less than K
<!-- ### Distances
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
Distances recieved over MQTT measured by duck J/K to each other duck -->

### Odometry
```
odometry/duck<J> -> odometry/duck<J>
odometry/duck<K> -> odometry/duck<K>
```
Odometry measurements from each other duck
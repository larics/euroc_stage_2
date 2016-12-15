# EUROC Vicon transmitter

This package enables Vicon odometry transmission from the Vicon ROS network to a
remote robot network.
It works based on a socket UDP connection.
A client socket node on the Vicon network serializes the messages and
continuously broadcasts them to all hosts on the EUROC network.
A server socket node on the EUROC network receives those messages, deserializes
them and broadcasts them to the EUROC ROS network.

```
--------------------------------------------------------------------------------
Vicon ROS network                        | EUROC ROS network
                                         |
/mav_name/vrpn/estimated_odometry        |
                                         |              computer (2)
                                         |  - receives serial data
                                         |  - publishes
                                         |   /mav_name/vrpn/estimated_odometry
--------------------------------------------------------------------------------
                    |                                        |             
                    |  receive                       send    |          
                    -------------- computer (1) --------------
                          - serializes Vicon odometry
                          - broadcast data to all hosts on EUROC network
```

## Setup
Connect both networks to a computer (1).
Make sure the Vicon network is recognized as default network (Linux ```route```)
such that ROS master will start on this network on this machine.

### Vicon Network Side
On computer (1) on the Vicon network start node_manager's ROS master discovery.
Run vicon.launch which starts the ROS VRPN client and the transmission node.
Alternatively start node manually with correct namespace and broadcast ip
```rosrun euroc_vicon_transmission vicon_transmitter_node```.

Set the correct MAV namespace through ```mav_name```.
Set ```broadcast_ip``` to the receiving broadcast IP on the EUROC network side.
E.g., for a network ```192.168.0.0``` and a netmask ```255.255.255.0``` the
broadcast IP is ```192.168.0.255```.
For a network ```10.10.0.0``` and a netmask ```255.255.0.0``` the broadcast IP
is ```10.10.255.255```.
For determining the IP and netmask run ```ifconfig```.

### EUROC Network Side
On computer (2) which is only connected to the EUROC network start euroc.launch
or alternatively ```rosrun euroc_vicon_transmission vicon_pub_node``` with
correct namespacing.
This starts a server socket which publishes all received messages to the EUROC
ROS network.

Set ```mav_name``` for correct namespacing.

## Warnings and Common Errors
If ```PROTOBUF_PROTOC_EXECUTABLE-NOTFOUND``` during compilation:
clean package euroc_vicon_transmisson build and run
```sudo apt-get install protobuf-compiler```

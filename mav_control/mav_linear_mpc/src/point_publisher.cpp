/**
 * Publish points to mpc controller
 */
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <stdio.h>


int main(int argc, char **argv){
    ros::init(argc, argv, "point_publisher");

    ros::NodeHandle n;

    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseStamped>("points", 1000);

    ros::Rate loop_rate(1);

    geometry_msgs::PoseStamped point;
    point.header.seq = 0;
    point.header.frame_id = "euroc_hex/base_link";
    point.pose.orientation.w = 1.0;

    float path[4][3] = {{0,0,1},
                        {0,3,2},
                        {3,3,3},
                        {3,0,2}};

    int count = 0;

    while(ros::ok()){
       
        for(int i = 0; i<sizeof(path)/sizeof(path[0]); i++){
        ros::Duration(10.0).sleep();
        point.pose.position.x = path[i][0];
        point.pose.position.y = path[i][1];
        point.pose.position.z = path[i][2];

        

        point_pub.publish(point);
        std::cout << "Point : " << point << std::endl;
             
        }
        
        
        //ros::Duration(2.0).sleep(); //spavaj 2s
        
    }
return 0;
}
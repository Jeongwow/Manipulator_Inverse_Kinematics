#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

int Arr[4];
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");

	ros::NodeHandle n;	

	ros::Subscriber sub3 = n.subscribe("array", 10, arrayCallback);

	ros::spin();


	return 0;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	// print all the remaining numbers
    printf("%.3f %.3f %.3f",array->data[0], array->data[1], array->data[2]);

	printf("\n");

	return;
}
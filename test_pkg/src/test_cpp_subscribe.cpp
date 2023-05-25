#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

int Arr[5];
void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "arraySubscriber");

	ros::NodeHandle n;	

	ros::Subscriber sub3 = n.subscribe("ik_position", 10, arrayCallback);
	ros::spin();


	printf("\n");
	return 0;
}

void arrayCallback(const std_msgs::Float32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;
	}
	for(int j = 1; j < 5; j++)
	{
		printf("%d, ", Arr[j]);
	}


	return;
}
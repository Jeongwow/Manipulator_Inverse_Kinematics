#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

int main(int argc, char **argv)
{
    

	ros::init(argc, argv, "arrayPublisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("array", 100);

    float randnum;
	while (ros::ok())
	{
		std_msgs::Float32MultiArray array;
		//Clear array
		array.data.clear();
		//for loop, pushing data in the size of the array
		for (int i = 0; i < 3; i++)
		{
			//assign array a random number between 0 and 255.
            randnum = 0.03;
			array.data.push_back(randnum);
            printf("%.f, ", randnum);
		}
        printf("\n");

		//Publish array
		pub.publish(array);
		//Let the world know
		ROS_INFO("I published something!");

		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
	}

}
//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>

//Uncomment this and replace {type} with the type of message when needed
#include <geometry_msgs/Point.h>

//changes these to the coordiantes of the points used in lab 9
double coords[5][3] = {{-1,0,0}, {-1,-2,0}, {3,-1.5,0}, {-1,-3,0}, {4,-3,0}};


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "human_sim");
  ros::NodeHandle n;

  //publisher to the human node
  ros::Publisher human_pub = n.advertise<geometry_msgs::Point>("human", 10);

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //point message variable
  geometry_msgs::Point position;

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {


    //gets a random integer between 0 and 4
    srand((unsigned)time(0));
    int randint = rand()%5;

    //sets the position to the random point
    position.x = coords[randint][0];
    position.y = coords[randint][1];
    position.z = 0.0;

    //publishes the position
    human_pub.publish(position);

    //sends out any data necessary then waits 2 seconds
    ros::spinOnce();
    sleep(2);

  }

  return 0;
}



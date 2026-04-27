#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

//Change these to the two corners of the rectangle for the crossing area
#define X1 0
#define Y1 0
#define X2 0.5
#define Y2 -8.5

//changes these to the coordiantes of the points used in lab 9
double coords[5][3] = {{-1,0,0}, {-1,-2,0}, {3,-1.5,0}, {-1,-3,0}, {4,-3,0}};

//message type for the point of the human
#include <geometry_msgs/Point.h>

//callback for the position of the human
void human_callback(const geometry_msgs::Point);

int main( int argc, char** argv )
{
  ros::init(argc, argv, "Visual_Updater");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //creates the subscriber for the humans position
  ros::Subscriber human_sub = n.subscribe("human", 1, human_callback);

  //loop rate of 10hz
  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    //creating the line strip marker, which is a series of lines and creating the points
    visualization_msgs::Marker line_strip, points;
    points.header.frame_id = line_strip.header.frame_id = "/map";
    line_strip.header.stamp = points.header.stamp = ros::Time::now();
    line_strip.ns = points.ns = "Visual_Updater";
    line_strip.action = points.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = points.pose.orientation.w = 1.0;
    line_strip.id = 1;
    points.id = 2;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    points.type = visualization_msgs::Marker::POINTS;

    // LINE_STRIP markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    //x and y are the width and height of the points
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // Line strip is yellow
    line_strip.color.r = 1.0;
    line_strip.color.g = 1.0;
    line_strip.color.b = 0.3;
    line_strip.color.a = 1.0;

    // points are green
    points.color.r = 0.0;
    points.color.g = 1.0;
    points.color.b = 0.0;
    points.color.a = 1.0;

    //create the point variable
    geometry_msgs::Point p;

    //set the first point to be X1,Y1
    p.x = X1;
    p.y = Y1;
    p.z = 0.0;
    //set the first line_strip point to be this point
    line_strip.points.push_back(p);

    //set the second point to be X2,Y1
    p.x = X2;
    p.y = Y1;
    line_strip.points.push_back(p);
    
    //set the third point to be X2,Y2
    p.x = X2;
    p.y = Y2;
    line_strip.points.push_back(p);

    //set the fourth point to be X1,Y2
    p.x = X1;
    p.y = Y2;
    line_strip.points.push_back(p);

    //set the last point to return to X1,Y1
    p.x = X1;
    p.y = Y1;
    line_strip.points.push_back(p);

    //set the points to be equal to the coordinates
    for(int i=0; i<5; i++)
    {
      p.x=coords[i][0];
      p.y=coords[i][1];
      points.points.push_back(p);
    }

    // wait for the marker to be subscribed to before publishing to insure it is created on the map
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      sleep(1);
    }

    //publish the markers
    marker_pub.publish(line_strip);
    marker_pub.publish(points);

    //wait forever to insure the callback function can be used
    while(ros::ok())
    {
      ros::spinOnce(); 
      loop_rate.sleep();
    }
  }
}

//callback function for the humans location which updates the circle on the map
void human_callback(const geometry_msgs::Point point)
{
  ros::NodeHandle h;
  ros::Publisher callback_marker_pub = h.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate loop_rate(10);

  //creates the circle marker
  visualization_msgs::Marker circle;
  circle.header.frame_id = "/map";
  circle.header.stamp = ros::Time::now();
  circle.ns = "Visual_Updater";
  circle.action = visualization_msgs::Marker::ADD;
  circle.pose.orientation.w = 1.0;
  circle.id = 0.0;
  circle.type = visualization_msgs::Marker::CYLINDER;
  circle.pose.orientation.w = 1.0;

  //set the size of the cylinder x and y are the radius z is the height
  circle.scale.x = 1.0;
  circle.scale.y = 1.0;
  circle.scale.z = 0.01;

  //set the color to red
  circle.color.r = 1.0;
  circle.color.g = 0.0;
  circle.color.b = 0.0;
  circle.color.a = 1.0;

  //set the center to the humans location
  circle.pose.position.x=point.x;
  circle.pose.position.y=point.y;
  circle.pose.position.z=0.0;

  //publish the circle
  callback_marker_pub.publish(circle);

  ros::spinOnce(); 
  loop_rate.sleep();

}

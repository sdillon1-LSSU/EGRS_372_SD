//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>

//These libraries are needed for the dynamic reconfiguring
//DoubleParameter could be substituted for IntParameter if ints needed to be set instead of doubles, this applies for all variables
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/service.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalID.h>
#include <string>
#include <signal.h>
#include <cmath>
#include <fstream>
#include <vector>
#include "egrs372_lab9/RobotParams.h"


//Uncomment this and replace {type} with the type of message when needed
//#include "std_msgs/{type}.h"

ros::Publisher goal_pub;
ros::Publisher cmd_pub;
ros::Publisher cancel_pub;
ros::Publisher param_pub;

bool bumper_pressed = false;
bool goal_reached = false;
bool goal_sent = false;

geometry_msgs::PoseStamped goal_1;
geometry_msgs::PoseStamped goal_2;
geometry_msgs::PoseStamped goal_3;
geometry_msgs::PoseStamped goal_4;
geometry_msgs::PoseStamped goal_5;

// =====================================================
// Robot states
// =====================================================

enum RobotState
{
  G1,
  G2,
  G3,
  G4,
  G5,
  Waiting
};

RobotState current_goal = Waiting;

std::string stateToString(RobotState s)
{
  switch (s)
  {
    case G1:    		 return "Goal_1";
    case G2:        	 return "Goal_2";
    case G3:      	 return "Goal_3";
    case G4:       	 return "Goal_4";
    case G5:   		 return "Goal_5";
    default:             return "UNKNOWN";
  }
}

struct Params
{
  double max_vel_x;
  bool allow_backward;
  double max_rot_vel;
  bool adjust_orientation;
};

std::vector<Params> param_list;

void applyParams(const Params& p)
{
  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse resp;
  dynamic_reconfigure::Config conf;

  dynamic_reconfigure::DoubleParameter param1;
  param1.name = "max_vel_x";
  param1.value = p.max_vel_x;

  dynamic_reconfigure::DoubleParameter param2;
  param2.name = "max_rot_vel";
  param2.value = p.max_rot_vel;

  conf.doubles.push_back(param1);
  conf.doubles.push_back(param2);

  // Handle backward motion
  dynamic_reconfigure::DoubleParameter min_vel;
  min_vel.name = "min_vel_x";
  min_vel.value = p.allow_backward ? -p.max_vel_x : 0.0;

  conf.doubles.push_back(min_vel);

  // Orientation tolerance
  dynamic_reconfigure::DoubleParameter yaw_tol;
  yaw_tol.name = "yaw_goal_tolerance";
  yaw_tol.value = p.adjust_orientation ? 0.17 : 10.0;

  conf.doubles.push_back(yaw_tol);

  req.config = conf;

  ros::service::call("/move_base/DWAPlannerROS/set_parameters", req, resp);
  
  egrs372_lab9::RobotParams msg;
  msg.max_vel_x = p.max_vel_x;
  msg.allow_backward = p.allow_backward;
  msg.max_rot_vel = p.max_rot_vel;
  msg.adjust_orientation = p.adjust_orientation;

  param_pub.publish(msg);

  ROS_INFO("Updated parameters applied");
}

geometry_msgs::PoseStamped makeGoal(double x, double y, double z, double w)
{
  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = z;
  goal.pose.orientation.w = w;
  return goal;
}

void sendGoal(const geometry_msgs::PoseStamped& goal, const std::string& name)
{
  geometry_msgs::PoseStamped msg = goal;
  msg.header.stamp = ros::Time::now();
  goal_pub.publish(msg);

  goal_sent = true;
  goal_reached = false;

  ROS_INFO("Sent goal: %s -> x=%.3f y=%.3f",
           name.c_str(),
           goal.pose.position.x,
           goal.pose.position.y);
}

void stopRobot()
{
  geometry_msgs::Twist stop_msg;
  stop_msg.linear.x = 0.0;
  stop_msg.linear.y = 0.0;
  stop_msg.linear.z = 0.0;
  stop_msg.angular.x = 0.0;
  stop_msg.angular.y = 0.0;
  stop_msg.angular.z = 0.0;
  
  for (int i = 0; i < 5; i++)
  {
    cmd_pub.publish(stop_msg);
    ros::Duration(0.05).sleep();
  }
}

void cancelMoveBaseGoal()
{
  actionlib_msgs::GoalID cancel_msg;
  cancel_pub.publish(cancel_msg);   // empty GoalID cancels all goals
  ros::Duration(0.1).sleep();
}

bool updateGoalsFromParams()
{
  static double last_L1_x = 0.0,  last_L1_y = 0.0,  last_L1_theta = 0.0;
  static double last_L2_x = 0.0,  last_L2_y = 0.0,  last_L2_theta = 0.0;
  static double last_L3_x = 0.0,  last_L3_y = 0.0,  last_L3_theta = 0.0;
  static double last_L4_x = 0.0,  last_L4_y = 0.0,  last_L4_theta = 0.0;
  static double last_L5_x = 0.0,  last_L5_y = 0.0,  last_L5_theta = 0.0;
 
  static bool first_time = true;

  double L1_x,  L1_y ,  L1_theta;
  double L2_x,  L2_y ,  L2_theta;
  double L3_x,  L3_y ,  L3_theta;
  double L4_x,  L4_y ,  L4_theta;
  double L5_x,  L5_y ,  L5_theta;

  ros::param::get("/L1/x", L1_x);
  ros::param::get("/L1/y", L1_y);
  ros::param::get("/L1/theta", L1_theta);
  
  ros::param::get("/L2/x", L2_x);
  ros::param::get("/L2/y", L2_y);
  ros::param::get("/L2/theta", L2_theta);
  
  ros::param::get("/L3/x", L3_x);
  ros::param::get("/L3/y", L3_y);
  ros::param::get("/L3/theta", L3_theta);
  
  ros::param::get("/L4/x", L4_x);
  ros::param::get("/L4/y", L4_y);
  ros::param::get("/L4/theta", L4_theta);
  
  ros::param::get("/L5/x", L5_x);
  ros::param::get("/L5/y", L5_y);
  ros::param::get("/L5/theta", L5_theta);


  bool changed =
      first_time; //||
     // home_x != last_home_x || home_y != last_home_y || home_theta != last_home_theta ||
     // pick_x != last_pick_x || pick_y != last_pick_y || pick_theta != last_pick_theta ||
     // place_x != last_place_x || place_y != last_place_y || place_theta != last_place_theta;

  if (changed)
  {
	  goal_1 = makeGoal(L1_x,  L1_y,  sin(L1_theta / 2.0),  cos(L1_theta / 2.0));
	  goal_2 = makeGoal(L2_x,  L2_y,  sin(L2_theta / 2.0),  cos(L2_theta / 2.0));
	  goal_3 = makeGoal(L3_x,  L3_y,  sin(L3_theta / 2.0),  cos(L3_theta / 2.0));
	  goal_4 = makeGoal(L4_x,  L4_y,  sin(L4_theta / 2.0),  cos(L4_theta / 2.0));
	  goal_5 = makeGoal(L5_x,  L5_y,  sin(L5_theta / 2.0),  cos(L5_theta / 2.0));
    

   // last_home_x = home_x;
   // last_home_y = home_y;
   // last_home_theta = home_theta;

   // last_pick_x = pick_x;
   // last_pick_y = pick_y;
  //  last_pick_theta = pick_theta;

   // last_place_x = place_x;
   // last_place_y = place_y;
//last_place_theta = place_theta;

    first_time = false;

    ROS_INFO("Updated Locations:");
   
  }

  return changed;
}

void bumperCallback(const std_msgs::Byte::ConstPtr& msg)
{
  bumper_pressed = (msg->data != 0);
}

void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  if (msg->status.status == 3) // SUCCEEDED
  {
    ROS_INFO("Goal reached!");
    goal_reached = true;
  }
}

int main(int argc, char **argv)
{

  //names the programs node
  ros::init(argc, argv, "Lab9");
  ros::NodeHandle n;
  
  goal_pub   = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  cmd_pub    = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  cancel_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
  param_pub = n.advertise<egrs372_lab9::RobotParams>("/robot_params", 10);
  ros::Subscriber result_sub = n.subscribe("/move_base/result", 10, resultCallback);
  ros::Subscriber bumper_sub  = n.subscribe("bumper", 10, bumperCallback);

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //creating the dynamic reconfigure variables
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  //setting max_vel_x
  double_param.name = "max_vel_x";
  double_param.value = 0.13;
  conf.doubles.push_back(double_param);

  //srv_req is whats actually sent to the service and is set to be equal to conf which should contain the parameters
  srv_req.config = conf;

  //calling the service to actually set the parameters
  ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

  //clearing conf so that it can be used later to set the parameters
  //if this is not done the values for max_vel_x and min_vel_x are still saved in conf
  conf.doubles.clear();
  
  updateGoalsFromParams();
  
 std::ifstream file("/home/egrs372/catkin_ws/src/egrs372_lab9/params.txt");
std::string line;

while (getline(file, line))
{
  std::stringstream ss(line);
  std::string val;

  Params p;

  getline(ss, val, ',');
  p.max_vel_x = atof(val.c_str());

  getline(ss, val, ',');
  p.allow_backward = (val == "true");

  getline(ss, val, ',');
  p.max_rot_vel = atof(val.c_str());

  getline(ss, val, ',');
  p.adjust_orientation = (val == "true");

  param_list.push_back(p);
}
  
   while (ros::ok())
  {
    ros::spinOnce();
  
   bool new_bump = (bumper_pressed);
  
  switch (current_goal)
    {
      case G1:
      {
        if (!goal_sent)
		{
		  applyParams(param_list[0]);
          sendGoal(goal_1, "Goal 1");
		}

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          current_goal = G2;
        }
        break;
      }

      case G2:
      {
         if (!goal_sent)
		 {
			 applyParams(param_list[1]);
          sendGoal(goal_2, "Goal 2");
		 }

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          current_goal = G3;
        }
        break;
      }

      case G3:
      {
        if (!goal_sent)
		{
			applyParams(param_list[2]);
          sendGoal(goal_3, "Goal 3");
		}

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          current_goal = G4;
        }
        break;
      }

      case G4:
      {
         if (!goal_sent)
		 {
			 applyParams(param_list[3]);
          sendGoal(goal_4, "Goal 4");
		 }

        if (goal_reached)
        {
          goal_reached = false;
          goal_sent = false;
          current_goal = G5;
        }
        break;
      }

      case G5:
      {
        if (!goal_sent)
		{
			applyParams(param_list[4]);
          sendGoal(goal_5, "Goal 5");
		}

        if (goal_reached)
        {
          
        }
        break;
      }
	  
	  case Waiting:
      {
       if (new_bump)
        {
		  goal_reached = false;
          goal_sent = false;
          current_goal = G1;
        }
        break;
      }
	  
      default:
        break;
    }

    loop_rate.sleep();
  }

  
  cancelMoveBaseGoal();
  stopRobot();
  return 0;
}


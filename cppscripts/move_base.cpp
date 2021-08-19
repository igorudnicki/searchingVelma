#include "ros/ros.h"
#include <sstream>
#include <cmath>
#include <vector>
#include <stdbool.h>
#include <stdexcept>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>

#include <costmap_2d/costmap_2d_ros.h>

#include <global_planner/planner_core.h>

#include <base_local_planner/goal_functions.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_datatypes.h>

#include "searcher/Stpt.h"

using namespace std;

costmap_2d::Costmap2DROS *local_costmap;
costmap_2d::Costmap2DROS *global_costmap;
tf2_ros::Buffer *globalBuf;
tf2_ros::Buffer *localBuf;
tf2_ros::TransformListener *localTf;
tf2_ros::TransformListener *globalTf;

ros::Publisher pub_cmd;
ros::Publisher pub_pose;
ros::Publisher pub_point;
ros::Publisher pub_path;

dwa_local_planner::DWAPlannerROS* local_planner;

global_planner::GlobalPlanner* global_planner_stpt;

geometry_msgs::PoseStamped start_pose;
vector<geometry_msgs::PoseStamped> planned_path;

bool inaction = false;

bool stpt_callback(searcher::Stpt::Request &req,searcher::Stpt::Response &resp);
string moveToStpt(geometry_msgs::PoseStamped &stpt);
void rotationBehaviour();
double getThetaQuat(geometry_msgs::PoseStamped pose);

int main(int argc,char **argv)
{
  ros::init(argc, argv, "move_base");
  cout<<"Working node\n";
  ros::NodeHandle n;
  ros::Rate loop_rate(50);
  ros::ServiceServer service = n.advertiseService("stpt_move", stpt_callback);

  pub_cmd = n.advertise<geometry_msgs::Twist>("mux_vel_nav/cmd_vel",10); //mux_vel_nav
  pub_pose = n.advertise<geometry_msgs::PoseStamped>("/pose",10);
  pub_point = n.advertise<geometry_msgs::Point>("/points",10);
  pub_path = n.advertise<nav_msgs::Path>("/pathplanned",10);

  localBuf = new tf2_ros::Buffer(ros::Duration(10),true);
  localTf = new tf2_ros::TransformListener(*localBuf);
  local_costmap = new costmap_2d::Costmap2DROS("local_costmap",*localBuf);


  local_planner = new dwa_local_planner::DWAPlannerROS();
  local_planner->initialize("local_planner",localBuf,local_costmap);
  if(local_planner->isInitialized())cout<<"local_planner init correctly\n";

  globalBuf = new tf2_ros::Buffer(ros::Duration(10),true);
  globalTf = new tf2_ros::TransformListener(*globalBuf);
  global_costmap = new costmap_2d::Costmap2DROS("global_costmap",*globalBuf);

  global_planner_stpt = new global_planner::GlobalPlanner("global_planner", global_costmap->getCostmap(),"map");


  global_costmap ->start();
  local_costmap->start();

  ros::spin();
  return 0;
  }

string moveToStpt(geometry_msgs::PoseStamped &stpt)
{
  double start =ros::Time::now().toSec();
  ros::Duration d(5);
  double secs = d.toSec();
  double end = start + secs; 
  ros::Rate loop_rate(20);

  if(!global_costmap->getRobotPose(start_pose)) 
    return "ERR_POSE\n";
  if(!global_planner_stpt->makePlan(start_pose,stpt,planned_path))
    return "ERR_PLAN_GLOB\n";
  if(!local_planner->setPlan(planned_path))
    return "ERR_PLAN_LOC\n";

  global_planner_stpt->publishPlan(planned_path);

  geometry_msgs::Twist cmd_msg ;
  static int counter;
  while(ros::ok())
  {
    inaction = true;

    if(!local_planner->computeVelocityCommands(cmd_msg))
    {
      counter++;
    }
    if(counter==10)
    {
      counter=0;
      cout<<"Enabling rotation behaviour\n";
      rotationBehaviour();

      start_pose.header.stamp = ros::Time(0);  

      if(!global_costmap->getRobotPose(start_pose)) 
        return "ERR_POSE\n";
      if(!global_planner_stpt->makePlan(start_pose,stpt,planned_path))
        return "ERR_PLAN_GLOB\n";
      if(!local_planner->setPlan(planned_path))
        return "ERR_PLAN_LOC\n";
      global_planner_stpt->publishPlan(planned_path);
    }

    if (local_planner->isGoalReached())
    {
      inaction = false;
      cmd_msg.linear.x = 0.0;
      cmd_msg.angular.z = 0.0;
      pub_cmd.publish(cmd_msg);
      return "GOAL_REACHED\n";
    }
    pub_pose.publish(start_pose);

    pub_cmd.publish(cmd_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return "FUNC_EXP";
}

bool stpt_callback(searcher::Stpt::Request &req,searcher::Stpt::Response &resp)
{
  tf2::Quaternion qt;
  geometry_msgs::PoseStamped stpt;
  ros::Rate loop_rate(20);

  cout<<"\nstpt_callback "<<local_costmap->getName()<<"\n";
  cout<<"Given x is "<<req.x<<"Given y is "<<req.y<<"Given theta is "<<req.theta<<"\n";

  stpt.header.frame_id="map";
  stpt.header.stamp = ros::Time(0);
  stpt.pose.position.x = req.x;
  stpt.pose.position.y = req.y;
  stpt.pose.position.z = 0.0;
  qt.setRPY(0,0,req.theta);
  stpt.pose.orientation.x = qt.getX();
  stpt.pose.orientation.y = qt.getY();
  stpt.pose.orientation.z = qt.getZ();
  stpt.pose.orientation.w = qt.getW();

  while(inaction)
  {
    loop_rate.sleep();
  }
  resp.result = moveToStpt(stpt);
  cout<<resp.result;

  return true;
}
void rotationBehaviour()
{
  double current_theta;
  geometry_msgs::Twist cmd_msg;

  cmd_msg.linear.x = 0.0;
  cmd_msg.angular.z = 0.0;
  pub_cmd.publish(cmd_msg);

  global_costmap->getRobotPose(start_pose);
  geometry_msgs::PoseStamped last_pose = start_pose;
  current_theta = getThetaQuat(start_pose);

  double start =ros::Time::now().toSec();
  ros::Duration d(1.5);
  double secs = d.toSec();
  double end = start + secs; 

  while(ros::Time::now().toSec()<end)
  {
      cmd_msg.angular.z = 0.6;
      pub_cmd.publish(cmd_msg);
  }
  global_costmap->getRobotPose(start_pose);
  if(fabs(getThetaQuat(start_pose)-getThetaQuat(last_pose)<0.15))
  {
    
    start =ros::Time::now().toSec();
    ros::Duration d(1.5);
    secs = d.toSec();
    end = start + secs; 
    while(ros::Time::now().toSec()<end)
    {
      cmd_msg.angular.z = -0.6;
      pub_cmd.publish(cmd_msg);
    }
      
  }
}
double getThetaQuat(geometry_msgs::PoseStamped pose)
{
  tf::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}
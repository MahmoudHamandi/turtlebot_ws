#include <math.h>
#include <vector>
#include <string>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "controller/TargetDistance.h"
#include "controller/Target.h"

using std::vector;
using std::string;
using controller::TargetDistance;
using controller::Target;
class TargetDist {
  public:
    TargetDist(): target_() {
        InitDistance();
      }

    void OdomCallback(const  nav_msgs::Odometry::ConstPtr& msg) {
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      double theta = msg->pose.pose.orientation.z;
      if (target_Set_) {
        double d_x = x-target_x_;
        double d_y = y-target_y_;
        TargetDistance tar;
        tar.rho = sqrt(pow(d_x,2)+pow(d_y,2));
        tar.alpha = -theta +atan2((d_y),(d_x));
        tar.beta = -theta - tar.alpha;
        target_ = tar;
      }
      else {
        this->InitDistance();
        target_x_ = x;
        target_y_ = y;
        ROS_INFO("No target is set");
      }
      ROS_INFO("r: %f, a: %f, b: %f",target_.rho,target_.alpha,target_.beta);
    }

    void TargetCallback(const controller::Target::ConstPtr& msg) {
      if(msg->follow) {
        this->SetTarget(msg->x, msg->y);
      }
      else {
        this->removeTarget();
      }
    }

    void SetTarget(double target_x, double target_y) {
      target_Set_ = true;
      target_x_ = target_x;
      target_y_ = target_y;
    }

    void removeTarget() {
      target_Set_ = false;
    }

    TargetDistance target_;

  private:
    double target_x_;
    double target_y_;
    bool target_Set_;
    void InitDistance() {
      target_Set_ = false;
      target_.rho = 0.0;
      target_.alpha = 0.0;
      target_.beta = 0.0;
    }

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::Publisher control = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity",1);
  TargetDist tar;
  //tar.SetTarget(1.0,1.0);
//  while (ros::ok()) {
  int count=0;
   //double target_x = 1;
   //double target_y = 1;
  //while (ros::ok())
  //{
  ros::Publisher pub = nh.advertise<Target>("target",10);
  ros::Subscriber sub = nh.subscribe("odom", 10, &TargetDist::OdomCallback, &tar);
  ros::Subscriber sub2 = nh.subscribe("target", 10, &TargetDist::TargetCallback, &tar);
  //ROS_INFO("out r: %f, a: %f, b: %f",tar.target_.rho,tar.target_.alpha,tar.target_.beta);
//    ros::Rate loop_rate(100);
  //  geometry_msgs::Twist cont;
  //  cont.angular.z = 0.1;
  //  cont.linear.x = 1.0;
  //  TargetDistance tar = OdomCall

  //  ROS_INFO("r: %f, a: %f, b: %f",);
  //  control.publish(cont);

//    ros::spinOnce();
  ros::spin(); 
//    loop_rate.sleep();
//    ++count;

//    if (count >100) {
//      tar.removeTarget();
//    }
//  }
  return 0;



}

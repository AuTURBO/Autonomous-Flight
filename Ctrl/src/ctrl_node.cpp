#include "ros/ros.h"

FiniteStateMachine* pFSM; //# https://www.youtube.com/watch?v=jaitqSU2HIA&ab_channel=BRicey

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "Ctrl");
  ros::NodeHandle nh("~");

  //# 쓸데없는 약어는 지양
  ros::Subscriber sub_odometry = nh.subscribe<nav_msgs::Odometry>("odometry",
                                                                   100,
                                                                   boost::bind(&Odom_Data_t::feed, &fsm.odom_data, _1),
                                                                   ros::VoidConstPtr(),
                                                                   ros::TransportHints().tcpNoDelay());

  ros::Subscriber sub_trajectory = nh.subscribe<quadrotor_msgs::PositionCommand>("trajectory",
                                                                                  100,
                                                                                  boost::bind(&Command_Data_t::feed, &fsm.cmd_data, _1),
                                                                                  ros::VoidConstPtr(),//# ???                                      
                                                                                  ros::TransportHints().tcpNoDelay());

  ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu",
                                                           100,
                                                           boost::bind(&Imu_Data_t::feed, &fsm.imu_data, _1),
                                                           ros::VoidConstPtr(),
                                                           ros::TransportHints().tcpNoDelay());
  while (ros::ok()) {
    
      
  }

  return 0;
}

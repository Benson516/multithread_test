#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>
/*
#include <thread>
#include <chrono>
*/
#include <boost/bind.hpp>

using std::vector;
using std::string;


void chatter_0_CB(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("[CB0]: [%s]", msg->data.c_str());
}

//
void chatter_prototype_CB(const std_msgs::String::ConstPtr& msg, string _topic_name)
{
    if (_topic_name.compare(string("chatter_5")) == 0){
        ros::Duration(1.2).sleep();
        // std::this_thread::sleep_for( std::chrono::milliseconds(1200) );
    }
    ROS_INFO("CB for [%s]: <%s>", _topic_name.c_str(), msg->data.c_str());
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener", ros::init_options::AnonymousName);

  // Handle with default namespace
  ros::NodeHandle n;

  // Total topic count
  size_t topic_count = 6;
  // Topic names
  vector<string> topic_names;
  for (size_t i=0; i < topic_count; ++i){
      std::stringstream _ss_topic_name;
      _ss_topic_name << "chatter_" << i;
      topic_names.push_back(_ss_topic_name.str());
  }
  // topic_names.push_back("chatter_0");



  // Subscribers
  vector<ros::Subscriber> subscriber_list;
  // Note: scpecify the message type explicitly for subscriber
  for (size_t i=0; i < topic_names.size(); ++i){
      subscriber_list.push_back( n.subscribe<std_msgs::String>( topic_names[i], 1000, boost::bind(chatter_prototype_CB, _1, topic_names[i]) ) );
  }
  /*
  ros::Subscriber sub_0 = n.subscribe("chatter_0", 1000, chatter_0_CB);
  */


  ros::AsyncSpinner spinner(12); // Use ? threads
  spinner.start();
  ros::waitForShutdown();

  // Spin
  // ros::spin();

  std::cout << "end\n";

  return 0;
}

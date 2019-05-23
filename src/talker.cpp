#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>

using std::vector;
using std::string;

#define TOPIC_COUNT 6

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker", ros::init_options::AnonymousName);

  // Handle with default namespace
  ros::NodeHandle n;

  // Total topic count
  size_t topic_count = TOPIC_COUNT;
  // Topic names
  vector<string> topic_names;
  for (size_t i=0; i < topic_count; ++i){
      std::stringstream _ss_topic_name;
      _ss_topic_name << "chatter_" << i;
      topic_names.push_back(_ss_topic_name.str());
  }
  // topic_names.push_back("chatter_0");


  // Publishers
  vector<ros::Publisher> _pubs;
  for (size_t i=0; i < topic_names.size(); ++i){
      _pubs.push_back(n.advertise<std_msgs::String>(topic_names[i], 1000));
  }
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter_1", 1000);



  double _loop_rate = 5.0; // 10.0; // 2.0
  ros::Rate loop_rate_obj(_loop_rate);
  int count = 0;
  while (ros::ok())
  {
      // Loop over all publishers
      for (size_t i=0; i < topic_names.size(); ++i){
          // Content of the message
          std_msgs::String msg;
          std::stringstream ss;
          ss << "[" << topic_names[i] << "] count = " << count;
          msg.data = ss.str();
          _pubs[i].publish(msg);
          //
          ROS_INFO("%s", msg.data.c_str());
      }
      std::cout << "--------------------\n";


    // SpinOnce
    ros::spinOnce();

    loop_rate_obj.sleep();
    ++count;
  }


  return 0;
}

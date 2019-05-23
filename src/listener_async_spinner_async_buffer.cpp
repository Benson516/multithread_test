#include "ros/ros.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
#include <vector>
#include <sstream>
#include <ctime>
/*
#include <thread>
#include <chrono>
*/
#include <boost/bind.hpp>

// async_buffer
#include <async_buffer.hpp>
// #include "async_buffer.hpp"
//

using std::vector;
using std::string;


#define TOPIC_COUNT 6


// async_buffer
size_t buffer_size = 10;
// async_buffer<string> buffer_1(buffer_size);
vector< async_buffer<string> > async_buffer_list(TOPIC_COUNT, async_buffer<string>(buffer_size) );

//


void chatter_0_CB(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("[CB0]: [%s]", msg->data.c_str());
}

//
void chatter_prototype_CB(const std_msgs::String::ConstPtr& msg, string _topic_name, size_t _topic_id)
{
    if (_topic_name.compare(string("chatter_5")) == 0){
        ros::Duration(1.2).sleep();
        // std::this_thread::sleep_for( std::chrono::milliseconds(1200) );
    }
    /*
    if (_topic_name.compare(string("chatter_1")) == 0){
        string _tmp_s;
        _tmp_s = msg->data;
        buffer_1.put(_tmp_s);
        // buffer_1.put(_topic_id);
    }
    */
    // ROS_INFO("CB for [%s]: <%s>", _topic_name.c_str(), msg->data.c_str());
    string _tmp_s;
    _tmp_s = msg->data;
    bool result = async_buffer_list[_topic_id].put( _tmp_s);

    if (!result){
        std::cout << _topic_name << ": buffer full.\n";
    }
}

//
bool test_copy_func(string &target, const string& source){
    target = source;
    target += " <<>>";
    return true;
}






int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener", ros::init_options::AnonymousName);

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



  // Subscribers
  vector<ros::Subscriber> subscriber_list;
  // Note: scpecify the message type expliciinttly for subscriber
  for (size_t i=0; i < topic_names.size(); ++i){
      subscriber_list.push_back( n.subscribe<std_msgs::String>( topic_names[i], 1000, boost::bind(chatter_prototype_CB, _1, topic_names[i], i) ) );
  }
  /*
  ros::Subscriber sub_0 = n.subscribe("chatter_0", 1000, chatter_0_CB);
  */




  // Test of assign_copy_func(), with costomized copy function
  async_buffer_list[1].assign_copy_func(&test_copy_func);
  //





  ros::AsyncSpinner spinner(12); // Use ? threads
  spinner.start();

  // ros::waitForShutdown();
  double loo_rate = 1.0;
  long long loop_time_ms = (long long)(1000.0/loo_rate); // 1 sec.
  ros::Rate loop_rate( 1000.0/float(loop_time_ms) ); // Hz
  //
  auto start_old = std::chrono::high_resolution_clock::now();;
  while (ros::ok()){
      //
      auto start = std::chrono::high_resolution_clock::now();

      // Evaluation
      //=============================================================//
      /*
      {
          int i = 1;
          std::pair<string,bool> _result_pair = buffer_1.front();
          if (_result_pair.second){
              std::cout << topic_names[i] << ": ";
              std::cout << "<" << _result_pair.first << ">\t";
              std::cout << "pop: " << buffer_1.pop();
              std::cout << "\n";
          }else{
              std::cout << topic_names[i] << ": ";
              std::cout << "empty\n";
          }
      }
      */

      for (size_t i=0; i < topic_names.size(); ++i){
          std::pair<string,bool> _result_pair = async_buffer_list[i].front();
          if (_result_pair.second){
              std::cout << topic_names[i] << ": ";
              std::cout << "buffer_size = " << async_buffer_list[i].size_est() << " ";
              std::cout << "msg:<" << _result_pair.first << ">\t";
              std::cout << "pop: " << async_buffer_list[i].pop();
              std::cout << "\n";
          }else{
              std::cout << topic_names[i] << ": ";
              std::cout << "empty\n";
          }
      }

      //=============================================================//
      // end Evaluation

      //
      auto elapsed = std::chrono::high_resolution_clock::now() - start;
      auto period = start - start_old;
      start_old = start;


      long long elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
      long long period_us = std::chrono::duration_cast<std::chrono::microseconds>(period).count();
      std::cout << "execution time (ms): " << elapsed_us*0.001 << ", ";
      std::cout << "period time error (ms): " << (period_us*0.001 - loop_time_ms) << "\n";


      //
      loop_rate.sleep();
  }


  // Spin
  // ros::spin();

  std::cout << "end\n";

  return 0;
}

#include <ROS_interface.hpp>

/*
#include <string>
#include <iostream>
#include <vector>
// #include <sstream>
#include <thread>
#include <chrono>
*/
using std::vector;
using std::string;


#define TOPIC_COUNT 6



int main(int argc, char **argv)
{
    ROS_INTERFACE ros_interface(argc, argv);



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

    /*
    // nickname for topic_id
    enum class MSG_ID{
        chatter_0,
        chatter_1,
        chatter_2,
        chatter_3,
        chatter_4,
        chatter_5
    };
    */
    // std::cout << "here\n";
    {
        using MSG::M_TYPE;
        for (size_t i=0; i < topic_names.size(); ++i){
            ros_interface.add_a_topic(topic_names[i], int(M_TYPE::String), true, 1000, 10);
        }
    }

    std::cout << "here\n";

    // start
    ros_interface.start();
    // std::this_thread::sleep_for( std::chrono::milliseconds(3000) );
    std::cout << "here\n";

    // Spin
    double _loop_rate = 2.0; //1.0;
    long long loop_time_ms = (long long)(1000.0/_loop_rate); // ms
    //
    auto start_old = std::chrono::high_resolution_clock::now();;
    while(ros_interface.is_running()){
        //
        auto start = std::chrono::high_resolution_clock::now();

        // Evaluation
        //=============================================================//
        for (size_t i=0; i < TOPIC_COUNT; ++i){
            std::cout << topic_names[i] << ": ";
            //
            string string_out;
            bool is_ok = ros_interface.get_string(i, string_out);
            if (is_ok){
                std::cout << "msg: <" << string_out << ">";
            }else{
                std::cout << "empty";
            }
            std::cout << "\n";
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


        // Added for releasing recource
        std::this_thread::sleep_for( std::chrono::milliseconds(loop_time_ms) );
    }

  return 0;
}

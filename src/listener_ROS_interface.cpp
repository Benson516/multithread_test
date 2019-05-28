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
using namespace cv;


#define STRING_TOPIC_COUNT 6



int main(int argc, char **argv)
{
    ROS_INTERFACE ros_interface(argc, argv);



    // Total topic count
    size_t topic_count = STRING_TOPIC_COUNT;
    // Topic names
    vector<string> string_topic_names;
    for (size_t i=0; i < topic_count; ++i){
        std::stringstream _ss_topic_name;
        _ss_topic_name << "chatter_" << i;
        string_topic_names.push_back(_ss_topic_name.str());
    }
    // string_topic_names.push_back("chatter_0");


    // nickname for topic_id
    enum class MSG_ID{
        chatter_0,
        chatter_1,
        chatter_2,
        chatter_3,
        chatter_4,
        chatter_5,

        camera_0,
        camera_1,
        camera_2,
        camera_3,
        camera_4,
        camera_5,
        camera_6,
        camera_7,
        camera_8,
    };

    // std::cout << "here\n";
    {
        using MSG::M_TYPE;
        // String
        for (size_t i=0; i < string_topic_names.size(); ++i){
            ros_interface.add_a_topic(string_topic_names[i], int(M_TYPE::String), true, 1000, 10);
        }
        // Image
        ros_interface.add_a_topic("/camera/1/0/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/1/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/1/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/2/image_sync", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/0/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/0/1/image", int(M_TYPE::Image), true, 1, 3);
        ros_interface.add_a_topic("/camera/2/2/image", int(M_TYPE::Image), true, 1, 3);
    }

    std::cout << "here\n";

    // start
    ros_interface.start();
    // std::this_thread::sleep_for( std::chrono::milliseconds(3000) );
    std::cout << "here\n";


    //
    int num_image = 9;
    vector<string> window_names;
    for (size_t i=0; i < num_image; ++i){
        std::stringstream _ss_window_name;
        _ss_window_name << "image_" << i;
        namedWindow(_ss_window_name.str(), cv::WINDOW_AUTOSIZE);
        window_names.push_back( _ss_window_name.str() );
    }






    // Spin
    double _loop_rate = 100.0; //1.0;
    long long loop_time_ms = (long long)(1000.0/_loop_rate); // ms
    //
    auto start_old = std::chrono::high_resolution_clock::now();;
    while(ros_interface.is_running()){
        //
        auto start = std::chrono::high_resolution_clock::now();

        // Evaluation
        //=============================================================//
        /*
        for (size_t i=0; i < STRING_TOPIC_COUNT; ++i){
            std::cout << string_topic_names[i] << ": ";
            //
            string string_out;
            bool is_ok = ros_interface.get_String(i, string_out);
            if (is_ok){
                std::cout << "msg: <" << string_out << ">";
            }else{
                std::cout << "empty";
            }
            std::cout << "\n";
        }
        */

        // Image
        int image_topic_id = int(MSG_ID::camera_0);
        /*
        for (size_t i=0; i < num_image; ++i){
            cv::Mat image_out;
            bool result = ros_interface.get_Image( (image_topic_id+i), image_out);

            if (result){
                imshow(window_names[i], image_out);
                // std::cout << "got one image\n";
                waitKey(1);
            }
        }
        */
        vector<cv::Mat> image_out_list(num_image);
        vector<bool> is_image_updated(num_image, false);
        for (size_t i=0; i < num_image; ++i){
            is_image_updated[i] = ros_interface.get_Image( (image_topic_id+i), image_out_list[i]);
        }
        for (size_t i=0; i < num_image; ++i){
            if (is_image_updated[i]){
                imshow(window_names[i], image_out_list[i]);
                // std::cout << "got one image\n";
                waitKey(1);
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


        // Added for releasing recource
        std::this_thread::sleep_for( std::chrono::milliseconds(loop_time_ms) );
    }

  return 0;
}

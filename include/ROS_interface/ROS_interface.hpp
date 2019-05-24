#ifndef GUI_ROS_INTERFACE_H
#define GUI_ROS_INTERFACE_H

// Determine if we are going to preint some debug information to std_out
#define __DEGUG__


// all_header.h
#include <all_header.h>

using std::vector;
using std::string;



class ROS_INTERFACE{
public:

    // The identifier for msg types
    enum class MSG_TYPE{
        String,
        Image,
        // PointCloud
    };


    // Constructors
    ROS_INTERFACE();

    // Methods

private:

    size_t _num_topics;


    // ROS subscribers
    image_transport::ImageTransport _ros_it;

    // Buffers
    // Note: each message type got an array of SPSC buffers with that type,
    //       each single topic use an unique SPSC buffer.
    //       The result is that we will have multiple arrays of SPSC buffers
    //---------------------------------------------------------//
    // String

    // Image

    // PointCloud

    //---------------------------------------------------------//

};




#endif

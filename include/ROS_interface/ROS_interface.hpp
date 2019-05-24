#ifndef GUI_ROS_INTERFACE_H
#define GUI_ROS_INTERFACE_H

// Determine if we are going to preint some debug information to std_out
#define __DEGUG__


// all_header.h
#include <all_header.h>



// Example:
// Note: the following thing should be done in main code
// The identifier for message global id
//-----------------------------------------//
// List the topic tokens in sequence wuth the same order as the topic names in topics name list
/*
enum class MSG_ID{
    chatter_0,
    chatter_1,
    chatter_2,
    chatter_3,
    chatter_4,
    chatter_5
};

vector<string> topic_name_list;
vector<int> topic_type_list;
vector<bool> topic_is_input_list;
vector<size_t> topic_buffer_length_list;

// Start adding
topic_name_list.push_back("/chatter_0");
topic_type_list.push_back(ROS_INTERFACE::String);
topic_is_input_list.push_back(true);
topic_buffer_length_list.push_back(10);

topic_name_list.push_back("/chatter_1");
topic_type_list.push_back(ROS_INTERFACE::String);
topic_is_input_list.push_back(true);
topic_buffer_length_list.push_back(10);

topic_name_list.push_back("/chatter_2");
topic_type_list.push_back(ROS_INTERFACE::String);
topic_is_input_list.push_back(true);
topic_buffer_length_list.push_back(10);
*/
//-----------------------------------------//


// library begins
//--------------------------------------------------//
namespace ROS_INTERFACE{
    // The identifier for message types
    // Note: the type of the enum is defaultly int
    enum class MSG_TYPE{
        String,
        Image,
        // PointCloud
    };
}// end of the namespace ROS_INTERFACE



class ROS_INTERFACE{
    // "using"
    using std::vector;
    using std::string;

public:


    // Constructors
    ROS_INTERFACE();
    // Setting up topics
    bool load_topics(
        const vector<string> &topic_name_list_in,
        const vector<int> &topic_type_list_in,
        const vector<bool> &topic_is_input_list_in,
        const vector<size_t> &topic_buffer_length_list_in);
    // Really start the ROS thread
    bool start();

    // Getting methods for each type of message
    // The topic_id should be the "global id"
    //---------------------------------------------------------//
    bool get_string(const int topic_id, string & content_out);
    //---------------------------------------------------------//

    // Sending methods for each type of message
    // The topic_id should be the "global id"
    //---------------------------------------------------------//
    bool send_string(const int topic_id, const string &content_in);
    //---------------------------------------------------------//

private:

    size_t _num_topics;

    // Although we only use "one" thread, by using this container,
    // we can start the thread later by push_back element
    vector<std::thread> _thread_list;
    void _ROS_worker();

    // List of topics parameters
    // TODO: make a containner to contain the following things in a single object for each topic
    //------------------------------//
    // Note: the following are setting be user
    vector<string> _topic_name_list; // mapping - topic_id : topic_name (string)
    vector<int> _topic_type_list;    // mapping - topic_id : topic_type (enum)
    vector<bool> _topic_is_input_list; // mapping - topic_id : is_input (bool)
    vector<size_t> _topic_buffer_length_list; // mapping - topic_id : buffer_length (size_t)
    //------------------------------//


    // List of topic tid in each type of message
    // Note: this "tid" or type_id is type specified, different from global topic_id
    //------------------------------//
    vector<int> _topic_tid_list; // mapping - topic_id : topic_type_id
    //------------------------------//




    // ROS image transportor (similar to  node handle, but for images)
    image_transport::ImageTransport _ros_it;


    // Subscribers

    // Publishers


    // SPSC Buffers
    //---------------------------------------------------------//
    // Note: each message type got an array of SPSC buffers with that type,
    //       each single topic use an unique SPSC buffer.
    //       The result is that we will have multiple arrays of SPSC buffers
    //---------------------------------------------------------//
    // String
    vector< async_buffer<string> > buffer_list_string;
    // Image

    // PointCloud

    //---------------------------------------------------------//

}; // end of the class ROS_INTERFACE





#endif

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

ROS_INTERFACE ros_interface;
// Method 1:
{
    using MSG::M_TYPE;
    ros_interface.add_a_topic("/chatter_0", M_TYPE::String, true, 10);
    ros_interface.add_a_topic("/chatter_1", M_TYPE::String, true, 10);
    ros_interface.add_a_topic("/chatter_2", M_TYPE::String, true, 10);
}

// Method 2:
std::vector<MSG::T_PARAMS> topic_param_list;
{
    using MSG::T_PARAMS;
    using MSG::M_TYPE;
    // Start adding topics
    topic_param_list.push_back( T_PARAMS("/chatter_0", M_TYPE::String, true, 10) );
    topic_param_list.push_back( T_PARAMS("/chatter_1", M_TYPE::String, true, 10) );
    topic_param_list.push_back( T_PARAMS("/chatter_2", M_TYPE::String, true, 10) );
}
ros_interface.load_topics(topic_param_list);
*/
//-----------------------------------------//


// library begins
//--------------------------------------------------//
namespace MSG{
    // The identifier for message types
    // Note: the type of the enum is defaultly int
    enum class M_TYPE{
        String,
        Image,
        // PointCloud
        NUM_MSG_TYPE
    };

    struct T_PARAMS{
        std::string name;
        int type; // According to the enum value defined in ROS_INTERFACE class
        bool is_input; // if not, it's output
        size_t buffer_length; // The buffer length setting for the SPSC buffer used for this topic
        size_t topic_id; // The topic_id
        //
        T_PARAMS(): type(0), is_input(false), buffer_length(1)
        {}
        T_PARAMS(const std::string &name_in, int type_in, bool is_input_in, size_t buffer_length_in):
            name(name_in), type(type_in), is_input(is_input_in), buffer_length(buffer_length_in)
        {}
    };
}// end of the namespace MSG



class ROS_INTERFACE{

public:


    // Constructors
    ROS_INTERFACE();
    // Setting up topics
    // Method 1: use add_a_topic to add a single topic one at a time
    // Method 2: use load_topics to load all topics
    bool add_a_topic(const std::string &name_in, int type_in, bool is_input_in, size_t buffer_length_in);
    bool load_topics(const std::vector<MSG::T_PARAMS> &topic_param_list_in);
    // Really start the ROS thread
    bool start();

    // Check if the ROS is started
    bool is_running();

    // Getting methods for each type of message
    // The topic_id should be the "global id"
    //---------------------------------------------------------//
    bool get_string(const int topic_id, std::string & content_out);
    //---------------------------------------------------------//

    // Sending methods for each type of message
    // The topic_id should be the "global id"
    //---------------------------------------------------------//
    bool send_string(const int topic_id, const std::string &content_in);
    //---------------------------------------------------------//

private:
    bool _is_started;
    size_t _num_topics;

    // Although we only use "one" thread, by using this container,
    // we can start the thread later by push_back element
    std::vector<std::thread> _thread_list;
    void _ROS_worker();

    // List of topics parameters
    // TODO: make a containner to contain the following things in a single object for each topic
    //------------------------------//
    // Note: the following are setting be user
    // mapping - topic_id : topic parameters (MSG::T_PARAMS)
    std::vector<MSG::T_PARAMS> _topic_param_list;
    //------------------------------//

    // List of topic tid in each type of message
    // The tid is mainly used for indexing SPSC buffers
    // Note 1: this "tid" or type_id is type specified, different from global topic_id
    // Note 2: These vetor will be filled at adding/loading topics
    //------------------------------//
    std::vector<int> _topic_tid_list; // mapping - topic_id : topic_type_id
    // Topic-param list for each message type
    std::vector< std::vector<MSG::T_PARAMS> > _msg_type_2_topic_params; // _msg_type_2_topic_params[type][tid] --> T_PARAMS
    //------------------------------//

    // The subs_id and pub_id for each topic_id
    // mapping - topic_id : subs_id/pub_id
    // The subs_id/pub_id is mainly used for indexing subscribers/publishers
    // Note: these ids as well as SPSC buffers will be generated when subscribing/advertising
    //------------------------------//
    std::vector<int> _subs_id_list;
    std::vector<int> _pub_id_list;
    //------------------------------//



    // ROS image transport (similar to  node handle, but for images)
    // image_transport::ImageTransport _ros_it;


    // Subscribers

    // Publishers


    // SPSC Buffers
    //---------------------------------------------------------//
    // Note: each message type got an array of SPSC buffers with that type,
    //       each single topic use an unique SPSC buffer.
    //       The result is that we will have multiple arrays of SPSC buffers
    //---------------------------------------------------------//
    // String
    std::vector< async_buffer<std::string> > buffer_list_string;
    // Image

    // PointCloud

    //---------------------------------------------------------//

}; // end of the class ROS_INTERFACE





#endif

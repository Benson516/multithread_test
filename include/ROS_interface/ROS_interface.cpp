#include <ROS_interface.hpp>

// using std::vector;
// using std::string;

// Constructors
ROS_INTERFACE::ROS_INTERFACE():
    _is_started(false),
    _num_topics(0),
    _msg_type_2_topic_params( size_t(MSG::M_TYPE::NUM_MSG_TYPE) )
{}



// Setting up topics
//----------------------------------------------------------------//
// Method 1: use add_a_topic to add a single topic one at a time
bool ROS_INTERFACE::add_a_topic(const std::string &name_in, int type_in, bool is_input_in, size_t buffer_length_in){
    _topic_param_list.push_back( MSG::T_PARAMS(name_in, type_in, is_input_in, buffer_length_in) );
    // Parsing parameters
    //----------------------------//
    _num_topics = _topic_param_list.size();
    // get topic_type_id and store them in separated arrays
    size_t i = _num_topics-1;
    _msg_type_2_topic_params[ _topic_param_list[i].type ].push_back( _topic_param_list[i] );
    _topic_tid_list[i] = ( _msg_type_2_topic_params[ _topic_param_list[i].type ].size() - 1 );
    //----------------------------//
    return true;
}
// Method 2: use load_topics to load all topics
bool ROS_INTERFACE::load_topics(const std::vector<MSG::T_PARAMS> &topic_param_list_in){
    // Filling the dataset inside the object
    // Note: Do not subscribe/advertise topic now
    //----------------------------//
    // Saving the parameters
    _topic_param_list = topic_param_list_in;
    // Parsing parameters
    //----------------------------//
    _num_topics = _topic_param_list.size();
    // get topic_type_id and store them in separated arrays
    for(size_t i=0; i < _topic_param_list.size(); ++i){
        _msg_type_2_topic_params[ _topic_param_list[i].type ].push_back( _topic_param_list[i] );
        _topic_tid_list[i] = ( _msg_type_2_topic_params[ _topic_param_list[i].type ].size() - 1 );
    }
    //----------------------------//
    return true;
}
//----------------------------------------------------------------//

// Really start the ROS thread
bool ROS_INTERFACE::start(){
    if (_is_started) return false; // We don't restart it again (which will result in multiple node, actually)
    // Start the ROS thread, really starting the ROS
    _thread_list.push_back( std::thread(&ROS_INTERFACE::_ROS_worker, this) );
    _is_started = true;
    return true;
}
bool ROS_INTERFACE::is_running(){
    return _is_started;
}

// Private methods
//---------------------------------------------//
void ROS_INTERFACE::_ROS_worker(){
    // This thread is called by start()

    // Subscribing topics: generate SPSC buffers,  generate _subs_id_list, subscribe
    // Note: the order of the above processes is important, since that the callback function should be exposed only when all the variables are set
    //----------------------------------//
    // Advertising topics: generate SPSC buffers, generated _pub_id_list, advertise
    // Note: the order of the above processes is important, since that the callback function should be exposed only when all the variables are set
    //----------------------------------//
    int _msg_type = 0;
    // String
    _msg_type = int(MSG::M_TYPE::String);
    for (size_t i=0; i < _msg_type_2_topic_params[_msg_type].size(); ++i){
        MSG::T_PARAMS _tmp_params = _msg_type_2_topic_params[_msg_type][i];
        if (_tmp_params.is_input){
            // Subscribe

        }else{
            // Publish
        }
    }
    //----------------------------------//

    // Start spinning and loop to the end



}

#include <ROS_interface.hpp>

// using std::vector;
// using std::string;

// Constructors
ROS_INTERFACE::ROS_INTERFACE(int argc, char **argv):
    _is_started(false),
    _num_topics(0),
    _msg_type_2_topic_params( size_t(MSG::M_TYPE::NUM_MSG_TYPE) )
{
    ros::init(argc, argv, "ROS_interface", ros::init_options::AnonymousName);
}



// Setting up topics
//----------------------------------------------------------------//
// Method 1: use add_a_topic to add a single topic one at a time
bool ROS_INTERFACE::add_a_topic(const std::string &name_in, int type_in, bool is_input_in, size_t ROS_queue_in, size_t buffer_length_in){
    // Add a topic
    size_t idx_new = _topic_param_list.size();
    _topic_param_list.push_back( MSG::T_PARAMS(name_in, type_in, is_input_in, ROS_queue_in, buffer_length_in, idx_new) );
    // Parsing parameters
    //----------------------------//
    _num_topics = _topic_param_list.size();
    // get topic_type_id and store them in separated arrays
    size_t i = _num_topics-1;
    _msg_type_2_topic_params[ _topic_param_list[i].type ].push_back( _topic_param_list[i] );
    _topic_tid_list.push_back( _msg_type_2_topic_params[ _topic_param_list[i].type ].size() - 1 );
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
        // Assign the topic_id
        _topic_param_list[i].topic_id = i;
        //
        _msg_type_2_topic_params[ _topic_param_list[i].type ].push_back( _topic_param_list[i] );
        _topic_tid_list.push_back( _msg_type_2_topic_params[ _topic_param_list[i].type ].size() - 1 );
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
    // _is_started = true;
    while(!_is_started){
        std::this_thread::sleep_for( std::chrono::milliseconds(200) );
    }
    return true;
}
bool ROS_INTERFACE::is_running(){
    if (!_is_started)
        return false;
    // else
    return ros::ok();
}

// Private methods
//---------------------------------------------//
void ROS_INTERFACE::_ROS_worker(){
    // This thread is called by start()


    // Handle with default namespace
    ros::NodeHandle _nh;

    // Subscribing topics: generate SPSC buffers,  generate _pub_subs_id_list, subscribe
    // Note: the order of the above processes is important, since that the callback function should be exposed only when all the variables are set
    //----------------------------------//
    // Advertising topics: generate SPSC buffers, generated _pub_subs_id_list, advertise
    // Note: the order of the above processes is important, since that the callback function should be exposed only when all the variables are set
    //----------------------------------//
    // id: -1 means not assigned
    _pub_subs_id_list.resize(_topic_param_list.size(), -1);
    //
    int _msg_type = 0;
    //

    // String
    _msg_type = int(MSG::M_TYPE::String);
    for (size_t _tid=0; _tid < _msg_type_2_topic_params[_msg_type].size(); ++_tid){
        MSG::T_PARAMS _tmp_params = _msg_type_2_topic_params[_msg_type][_tid];
        // SPSC Buffer
        buffer_list_string.push_back( async_buffer<std::string>( _tmp_params.buffer_length ) );
        // subs_id, pub_id
        if (_tmp_params.is_input){
            // Subscribe
            _pub_subs_id_list[_tmp_params.topic_id] = _subscriber_list.size();
            _subscriber_list.push_back( _nh.subscribe<std_msgs::String>( _tmp_params.name, _tmp_params.ROS_queue, boost::bind(&ROS_INTERFACE::_String_CB, this, _1, _tmp_params)  ) );
        }else{
            // Publish
            _pub_subs_id_list[_tmp_params.topic_id] = _publisher_list.size();
            _publisher_list.push_back( _nh.advertise<std_msgs::String>( _tmp_params.name, _tmp_params.ROS_queue) );
        }
    }

    // Image


    //----------------------------------//

    // Start spinning and loop to the end
    ros::AsyncSpinner spinner(6); // Use ? threads
    spinner.start();
    _is_started = true; // The flag for informing the other part of system that the ROS has begun.
    std::cout << "ros_iterface started\n";
    //
    ros::waitForShutdown();

    //
    // _is_started = false;
    std::cout << "End of ros_iterface\n";
}



// Callbacks and public method of each message type
//---------------------------------------------------------------//

// String
//---------------------------------------------------------------//
void ROS_INTERFACE::_String_CB(const std_msgs::String::ConstPtr& msg, const MSG::T_PARAMS & params){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[params.topic_id];
    //------------------------------------//
    //
    string _tmp_s;
    _tmp_s = msg->data;
    bool result = buffer_list_string[ _tid ].put( _tmp_s);

    if (!result){
        std::cout << params.name << ": buffer full.\n";
    }
}
bool ROS_INTERFACE::get_string(const int topic_id, std::string & content_out){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
     std::pair<string,bool> _result_pair = buffer_list_string[_tid].front(true);
     if (_result_pair.second){
         // front and pop
         content_out = _result_pair.first;
         return true;
     }else{
         // empty
         return false;
     }
}
bool ROS_INTERFACE::send_string(const int topic_id, const std::string &content_in){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    // Content of the message
    std_msgs::String msg;
    msg.data = content_in;
    _publisher_list[ _tid ].publish(msg);
    //
    ROS_INFO("%s", msg.data.c_str());
}
//---------------------------------------------------------------//

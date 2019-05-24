#include <ROS_interface.hpp>

using std::vector;
using std::string;

// Constructors
ROS_INTERFACE::ROS_INTERFACE()
{
    _num_topics = 0;


}

// Setting up topics
bool load_topics(
    const vector<string> &topic_name_list_in,
    const vector<int> &topic_type_list_in,
    const vector<bool> &topic_is_input_list_in,
    const vector<size_t> &topic_buffer_length_list_in)
{
    // Filling the dataset inside the object
    // Note: Do not subscribe/advertise topic now

    // Saving the parameters
    _topic_name_list = topic_name_list_in;
    _topic_type_list = topic_type_list_in;
    _topic_is_input_list = topic_is_input_list_in;
    _topic_buffer_length_list = topic_buffer_length_list_in;


    // Parsing parameters
    //----------------------------//
    _num_topics = topic_name_list_in.size();
    
    // get topic_type_id

    // generate SPSC buffers


    //----------------------------//

    return true;

}


// Really start the ROS thread
bool ROS_INTERFACE::start(){

    // Start the ROS thread, really starting the ROS
    _thread_list.push_back( std::thread(_ROS_worker) );
}

// Private methods
//---------------------------------------------//
void ROS_INTERFACE::_ROS_worker(){
    // This thread is called by start()

    // Subscribing topics

    // Advertising topics



    // Start spinning and loop to the end



}

#include <ROS_interface.hpp>

// using std::vector;
// using std::string;



// Constructors
ROS_INTERFACE::ROS_INTERFACE(int argc, char **argv):
    _is_started(false),
    _num_topics(0),
    _msg_type_2_topic_params( size_t(MSG::M_TYPE::NUM_MSG_TYPE) )
    // The temporary containers
    // _cloud_tmp_ptr (new pcl::PointCloud<pcl::PointXYZI>)
{
    //
    _num_ros_cb_thread = 6; // Use 6 threads
    //
    ros::init(argc, argv, "ROS_interface", ros::init_options::AnonymousName);
}
// Destructor
ROS_INTERFACE::~ROS_INTERFACE(){
    for (size_t i=0; i < _image_subscriber_list.size(); ++i){
        _image_subscriber_list[i].shutdown();
    }
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
    // ROS image transport (similar to  node handle, but for images)
    image_transport::ImageTransport _ros_it(_nh);

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
        buffer_list_String.push_back( async_buffer<std::string>( _tmp_params.buffer_length ) );
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
    _msg_type = int(MSG::M_TYPE::Image);
    for (size_t _tid=0; _tid < _msg_type_2_topic_params[_msg_type].size(); ++_tid){
        MSG::T_PARAMS _tmp_params = _msg_type_2_topic_params[_msg_type][_tid];
        // SPSC Buffer
        buffer_list_Image.push_back( async_buffer<cv::Mat>( _tmp_params.buffer_length ) );
        // Note: assign copy function
        buffer_list_Image[buffer_list_Image.size()-1].assign_copy_func( &ROS_INTERFACE::_cv_Mat_copy_func );
        //
        // subs_id, pub_id
        if (_tmp_params.is_input){
            // Subscribe
            _pub_subs_id_list[_tmp_params.topic_id] = _image_subscriber_list.size();
            _image_subscriber_list.push_back( _ros_it.subscribe( _tmp_params.name, _tmp_params.ROS_queue, boost::bind(&ROS_INTERFACE::_Image_CB, this, _1, _tmp_params)  ) );
        }else{
            // Publish
            _pub_subs_id_list[_tmp_params.topic_id] = _image_publisher_list.size();
            _image_publisher_list.push_back( _ros_it.advertise( _tmp_params.name, _tmp_params.ROS_queue) );
        }
    }

    // ITRIPointCloud
    _msg_type = int(MSG::M_TYPE::ITRIPointCloud);
    for (size_t _tid=0; _tid < _msg_type_2_topic_params[_msg_type].size(); ++_tid){
        MSG::T_PARAMS _tmp_params = _msg_type_2_topic_params[_msg_type][_tid];
        // SPSC Buffer
        buffer_list_ITRIPointCloud.push_back( async_buffer< pcl::PointCloud<pcl::PointXYZI> >( _tmp_params.buffer_length ) );
        // subs_id, pub_id
        if (_tmp_params.is_input){
            // Subscribe
            _pub_subs_id_list[_tmp_params.topic_id] = _subscriber_list.size();
            _subscriber_list.push_back( _nh.subscribe<msgs::PointCloud>( _tmp_params.name, _tmp_params.ROS_queue, boost::bind(&ROS_INTERFACE::_ITRIPointCloud_CB, this, _1, _tmp_params)  ) );
        }else{
            // Publish
            _pub_subs_id_list[_tmp_params.topic_id] = _publisher_list.size();
            _publisher_list.push_back( _nh.advertise<msgs::PointCloud>( _tmp_params.name, _tmp_params.ROS_queue) );
        }
    }

    //----------------------------------//

    // Start spinning and loop to the end
    ros::AsyncSpinner spinner(_num_ros_cb_thread); // Use ? threads
    spinner.start();
    _is_started = true; // The flag for informing the other part of system that the ROS has begun.
    std::cout << "ros_iterface started\n";
    //
    ros::waitForShutdown();


/*
    // Loop
    double _loop_rate = 100.0; //1.0;
    long long loop_time_ms = (long long)(1000.0/_loop_rate); // ms
    ros::Rate loop_rate_obj( 1000.0/float(loop_time_ms) ); // Hz
    //
    auto start_old = std::chrono::high_resolution_clock::now();;
    while (ros::ok()){
        //
        auto start = std::chrono::high_resolution_clock::now();

        // Evaluation
        //=============================================================//
        // pub all String
        bool is_published = false;
        is_published |= _String_pub();

        //=============================================================//
        // end Evaluation

        //
        auto elapsed = std::chrono::high_resolution_clock::now() - start;
        auto period = start - start_old;
        start_old = start;

        long long elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        long long period_us = std::chrono::duration_cast<std::chrono::microseconds>(period).count();


        if (is_published){
            std::cout << "(pub loop) execution time (ms): " << elapsed_us*0.001 << ", ";
            std::cout << "period time error (ms): " << (period_us*0.001 - loop_time_ms) << "\n";
        }

        //
        loop_rate_obj.sleep();
    }// end of Loop
*/


    //
    // _is_started = false;
    std::cout << "End of ros_iterface\n";
}






// Callbacks and public method of each message type
//---------------------------------------------------------------//

// String
//---------------------------------------------------------------//
// input
void ROS_INTERFACE::_String_CB(const std_msgs::String::ConstPtr& msg, const MSG::T_PARAMS & params){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[params.topic_id];
    //------------------------------------//
    //
    string _tmp_s;
    _tmp_s = msg->data;
    bool result = buffer_list_String[ _tid ].put( _tmp_s);

    if (!result){
        std::cout << params.name << ": buffer full.\n";
    }
}
bool ROS_INTERFACE::get_String(const int topic_id, std::string & content_out){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    return ( buffer_list_String[_tid].front(content_out, true) );
    /*
    // Legacy API
     std::pair<string,bool> _result_pair = buffer_list_String[_tid].front(true);
     if (_result_pair.second){
         // front and pop
         content_out = _result_pair.first;
         return true;
     }else{
         // empty
         return false;
     }
     */
}
// output
/*
bool ROS_INTERFACE::_String_pub(){
    bool is_published = false;
    // Loop over
    for (size_t _tid=0; _tid < buffer_list_String.size(); ++_tid){
        if (_msg_type_2_topic_params[int(MSG::M_TYPE::String)][_tid].is_input)
            continue;
        size_t topic_id = _msg_type_2_topic_params[int(MSG::M_TYPE::String)][_tid].topic_id;
        // else, it's output
        std::pair<string,bool> _result_pair = buffer_list_String[_tid].front(true);
        if (_result_pair.second){
            // front and pop
            // Content of the message
            std_msgs::String msg;
            msg.data = _result_pair.first;
            _publisher_list[ _pub_subs_id_list[topic_id] ].publish(msg);
            is_published = true;
            //
            // ROS_INFO("%s", msg.data.c_str());
        }else{
            // empty
        }
        //
    }
    //
    return is_published;
}
*/
bool ROS_INTERFACE::send_string(const int topic_id, const std::string &content_in){
    // pub_subs_id
    //------------------------------------//
    int _ps_id = _pub_subs_id_list[topic_id];
    //------------------------------------//
    // Content of the message
    std_msgs::String msg;
    msg.data = content_in;
    _publisher_list[ _ps_id ].publish(msg);
    //
    // ROS_INFO("%s", msg.data.c_str());

/*
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    bool result = buffer_list_String[ _tid ].put( content_in);
    if (!result){
        std::cout << _topic_param_list[topic_id].name << ": buffer full.\n";
    }
*/

}
//---------------------------------------------------------------//

// Image
//---------------------------------------------------------------//
// input
void ROS_INTERFACE::_Image_CB(const sensor_msgs::ImageConstPtr& msg, const MSG::T_PARAMS & params){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[params.topic_id];
    //------------------------------------//

    //
    // Get the (raw) image
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    bool result = buffer_list_Image[ _tid ].put( cv_ptr->image);

    if (!result){
        std::cout << params.name << ": buffer full.\n";
    }
}
bool ROS_INTERFACE::get_Image(const int topic_id, cv::Mat & content_out){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    return ( buffer_list_Image[_tid].front(content_out, true) );
}
// output
bool ROS_INTERFACE::send_Image(const int topic_id, const cv::Mat &content_in){
    // pub_subs_id
    //------------------------------------//
    int _ps_id = _pub_subs_id_list[topic_id];
    //------------------------------------//
    // Content of the message
    cv_bridge::CvImagePtr cv_ptr;
    _cv_Mat_copy_func(cv_ptr->image, content_in);
    _image_publisher_list[ _ps_id ].publish(cv_ptr->toImageMsg());
}
//---------------------------------------------------------------//


// ITRIPointCloud
//---------------------------------------------------------------//
// input
void ROS_INTERFACE::_ITRIPointCloud_CB(const msgs::PointCloud::ConstPtr& msg, const MSG::T_PARAMS & params){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[params.topic_id];
    //------------------------------------//
    // tmp cloud
    // Note 1: this have been moved to be a member of the class, so that it won't keep constructing and destructing.
    // Note 2: the pointer changed to use the std::shared_ptr instead of the original boost pointer

    // pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_tmp_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    if (!_cloud_tmp_ptr){
        _cloud_tmp_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
    // Conversion
    //-------------------------//
    _cloud_tmp_ptr->width = msg->pointCloud.size();
    // std::cout << "cloud size = " << _cloud_tmp_ptr->width << "\n";
    _cloud_tmp_ptr->height = 1;
    _cloud_tmp_ptr->is_dense = false;
    //
    // if (_cloud_tmp_ptr->points.size() < msg->pointCloud.size())
        _cloud_tmp_ptr->points.resize( msg->pointCloud.size() );
    // #pragma omp parallel for
    for (long long i = 0; i < _cloud_tmp_ptr->width; ++i)
    {
        _cloud_tmp_ptr->points[i].x = msg->pointCloud[i].x;
        _cloud_tmp_ptr->points[i].y = msg->pointCloud[i].y;
        _cloud_tmp_ptr->points[i].z = msg->pointCloud[i].z;
        _cloud_tmp_ptr->points[i].intensity = msg->pointCloud[i].intensity;
    }
    //-------------------------//
    // Add to buffer
    // bool result = buffer_list_ITRIPointCloud[ _tid ].put( *_cloud_tmp_ptr);
    bool result = buffer_list_ITRIPointCloud[ _tid ].put( _cloud_tmp_ptr); // Put in the pointer directly
    //
    if (!result){
        std::cout << params.name << ": buffer full.\n";
    }
}
bool ROS_INTERFACE::get_ITRIPointCloud(const int topic_id, pcl::PointCloud<pcl::PointXYZI> & content_out){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    return ( buffer_list_ITRIPointCloud[_tid].front(content_out, true) );
}
bool ROS_INTERFACE::get_ITRIPointCloud(const int topic_id, std::shared_ptr< pcl::PointCloud<pcl::PointXYZI> > & content_out_ptr){
    // Type_id
    //------------------------------------//
    int _tid = _topic_tid_list[topic_id];
    //------------------------------------//
    return ( buffer_list_ITRIPointCloud[_tid].front(content_out_ptr, true) );
}
bool ROS_INTERFACE::send_ITRIPointCloud(const int topic_id, const pcl::PointCloud<pcl::PointXYZI> &content_in){
    // pub_subs_id
    //------------------------------------//
    int _ps_id = _pub_subs_id_list[topic_id];
    //------------------------------------//

    // Content of the message
    msgs::PointCloud msg;
    // Conversion
    //-------------------------//
    msg.pointCloud.resize( content_in.points.size() );
    // #pragma omp parallel for
    for (size_t i = 0; i < content_in.width; ++i)
    {
        msg.pointCloud[i].x = content_in.points[i].x;
        msg.pointCloud[i].y = content_in.points[i].y;
        msg.pointCloud[i].z = content_in.points[i].z;
        msg.pointCloud[i].intensity = content_in.points[i].intensity;
    }
    //-------------------------//
    _publisher_list[ _ps_id ].publish(msg);
}
//---------------------------------------------------------------//

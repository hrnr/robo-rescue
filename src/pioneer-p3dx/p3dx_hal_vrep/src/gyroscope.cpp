#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <string>
#include <vector>


// publisher of Accelerometer data
ros::Publisher publisher;
std::string tf_prefix;

void callback_val(const std_msgs::String::ConstPtr & msg) {
    std::vector<double> axis_data;
    std::string data=msg->data;
    // linear parser of data in format x;x;x were x is a float
    std::string value=""; // partial data (x)
    for(size_t i=0; i<data.length();++i){
        if(data[i] == ';'){
            // in input data is 0,111 change to 0.111
            std::replace(value.begin(),value.end(), ',','.');
            axis_data.push_back(std::stof(value));
            value.clear();
            ++i;
        }
        value+=data[i];
    }
    // last element is added
    if(!value.empty()) axis_data.push_back(std::stof(value));
    if (axis_data.size() > 3){
        ROS_ERROR("HAL(VREP): Gyroscope received more than three values(axis) from vrep");
        axis_data.resize(3);
    }
    // creating new message
    geometry_msgs::Vector3Stamped msg_out;
    std_msgs::Header header; // creating header
    header.stamp = ros::Time::now(); // current time of data collection
    header.frame_id = tf_prefix + "/gyroSensors/sensor0";
    //  fill msg
    msg_out.header = move(header);
    msg_out.vector.x=axis_data[0];
    msg_out.vector.y=axis_data[1];
    msg_out.vector.z=axis_data[2];
    // publish vector of floats. One float for each axis X,Y,Z
    publisher.publish( move(msg_out)); 
}

int main(int argc, char ** argv) {
    // init node name as accelerometer
    ros::init(argc, argv, "gyroSensors");
    // set relative node namespace
    ros::NodeHandle n("~");

    // init parameters
    // resolve `tf_prefix` and `vrep_index` in parent namespaces
    int vrep_no = 0; // robot's index in vrep (if multiple instances)

    std::string tf_prefix_path;
    if (n.searchParam("tf_prefix", tf_prefix_path))
    {
        n.getParam(tf_prefix_path, tf_prefix);
    }

    std::string vrep_index_path;
    if (n.searchParam("vrep_index", vrep_index_path))
    {
        n.getParam(vrep_index_path, vrep_no);
    }

    // read msg from vrep topic
    ros::Subscriber sub_val = n.subscribe("/vrep/i" + std::to_string(vrep_no) +
        "_Pioneer_p3dx_GyroSensor", 1000, callback_val);
    // creates publisher in format ./sensor0/axis_data
    publisher = n.advertise<geometry_msgs::Vector3Stamped>("sensor0/axis_data", 1000);
    ROS_INFO("HAL(VREP): Gyroscope node initialized");
    
    // run event loop
    ros::spin();
    return 0;
}

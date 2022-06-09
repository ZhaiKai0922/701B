#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include <./gps_data.hpp>

class GPSDemo{
    public:
    GPSDemo(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    void ParseData(std::deque<GPSData>& deque_gps_data);

    private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GPSData> new_gps_data_;

    std::mutex buff_mutex_;
};
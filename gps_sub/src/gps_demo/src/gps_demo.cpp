#include <../include/gps_demo.hpp>
#include <glog/logging.h>

GPSDemo::GPSDemo(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh){
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GPSDemo::msg_callback, this);
    }

void GPSDemo::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr){
    buff_mutex_.lock();

    GPSData gps_data;
    gps_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gps_data.latitude = nav_sat_fix_ptr->latitude;
    gps_data.longitude = nav_sat_fix_ptr->longitude;
    gps_data.altitude = nav_sat_fix_ptr->altitude;

    new_gps_data_.push_back(gps_data);
    buff_mutex_.unlock();
}

void GPSDemo::ParseData(std::deque<GPSData>& gps_data_buff){
    buff_mutex_.lock();

    if(new_gps_data_.size() > 0){
        gps_data_buff.insert(gps_data_buff.end(), new_gps_data_.begin(), new_gps_data_.end());
        new_gps_data_.clear();
    }

    buff_mutex_.unlock();
}
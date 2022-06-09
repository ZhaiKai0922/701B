/*
 * @Description: 订阅velocity数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

#include "lidar_localization/sensor_data/velocity_data.hpp"

namespace lidar_localization {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const nav_msgs::OdometryConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};
}
#endif
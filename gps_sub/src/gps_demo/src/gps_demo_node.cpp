#include <../include/gps_demo_flow.hpp>
#include <ros/ros.h>
#include <glog/logging.h>

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/zk/ZK/urban_ws/gps_sub/src/gps_demo/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "gps_demo_node");
    ros::NodeHandle nh;

    std::shared_ptr<GpsDemoFlow> gps_demo_ = std::make_shared<GpsDemoFlow>(nh, "/navsat/fix", "/home/zk/ZK/urban_ws/gps_sub/src/gps_demo");

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();

        gps_demo_ -> Run();

        rate.sleep();
    }

    return 0;
}
#include <./gps_demo.hpp>
#include <memory>
#include <Eigen/Dense>
#include <string>
#include <fstream>

class GpsDemoFlow{
    public:
    GpsDemoFlow(ros::NodeHandle& nh, std::string cloud_topic, std::string data_path);

    bool Run();

    private:
    bool InitDataPath(const std::string& data_path);
    bool ReadData();
    bool InitGps();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool SavePose(std::ofstream& ofs, const Eigen::Matrix4f& Pose);

    private:
    std::shared_ptr<GPSDemo> gps_sub_ptr_;
    std::deque<GPSData> gps_data_buff_;

    GPSData current_gps_data_;

    Eigen::Matrix4f gps_pose_ = Eigen::Matrix4f::Identity();

    std::ofstream ground_truth_ofs_;

};

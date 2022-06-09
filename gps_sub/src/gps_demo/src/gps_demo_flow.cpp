#include <../include/gps_demo_flow.hpp>
#include <../include/file_manager.hpp>

GpsDemoFlow::GpsDemoFlow(ros::NodeHandle& nh, std::string topic_name, std::string data_path){
    gps_sub_ptr_ = std::make_shared<GPSDemo>(nh, topic_name, 1000);

    InitDataPath(data_path);
}

bool GpsDemoFlow::Run(){
    if(!ReadData()){
        return false;
    }
    if(!InitGps()){
        return false;
    }

    while(HasData()){
        if(!ValidData()){
            continue;
        }

        TransformData();
    }

    return true;
}

bool GpsDemoFlow::InitDataPath(const std::string& data_path){
    std::string traj_path = data_path + "/trajectory";
    if(!FileManager::CreateDirectory(traj_path, "GPS trajectory")){
        return false;
    }

    if(!FileManager::CreateFile(ground_truth_ofs_, traj_path + "/fix_ground_truth.txt")){
        return false;
    }
    
    return true;
}


bool GpsDemoFlow::ReadData(){
    gps_sub_ptr_ -> ParseData(gps_data_buff_);

    if(gps_data_buff_.size() == 0){
        return false;
    }

    return true;
}

bool GpsDemoFlow::InitGps(){
    static bool gps_inited = false;
    if(!gps_inited){
        GPSData gps_data = gps_data_buff_.front();
        gps_data.InitOriginPosition();
        gps_inited = true;
    }

    return gps_inited;
}

bool GpsDemoFlow::HasData(){
    if(gps_data_buff_.size() == 0){
        return false;
    }

    return true;
}

bool GpsDemoFlow::ValidData(){
    current_gps_data_= gps_data_buff_.front();

    gps_data_buff_.pop_front();

    return true;
}

bool GpsDemoFlow::TransformData(){
    gps_pose_ = Eigen::Matrix4f::Identity();

    current_gps_data_.UpdateXYZ();
    gps_pose_(0, 3) = current_gps_data_.local_E;
    gps_pose_(1, 3) = current_gps_data_.local_N;
    gps_pose_(2, 3) = current_gps_data_.local_U;

    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    lidar_to_imu_ << 0.999998, -0.000785403, 0.00202441, 0.810544,
                                         0.000755307, 0.99989, 0.0148245, -0.307054,
                                         -0.00203583, -0.014823, 0.999888, 0.802724,
                                         0, 0, 0, 1;
    gps_pose_ *= lidar_to_imu_;
    //output traj
    SavePose(ground_truth_ofs_, gps_pose_);
}

bool GpsDemoFlow::SavePose(std::ofstream& ofs, const Eigen::Matrix4f& pose){
    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 4; ++j){
            ofs << pose(i, j);

            if(i == 2 && j == 3){
                ofs << std::endl;
            }else{
                ofs << " ";
            }
        }
    }

    return true;
}
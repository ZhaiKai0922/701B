/*
 * @Description: NDT 匹配模块
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 */
#include "lidar_localization/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()),
    icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()){
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);

    icp_ptr_->setMaxCorrespondenceDistance(1.2);  //1.2
    icp_ptr_->setTransformationEpsilon(0.01);
    icp_ptr_->setEuclideanFitnessEpsilon(0.36);
    icp_ptr_->setMaximumIterations(50);  //100
    std::cout << "******Set ICP params******" << std::endl<< std::endl << std::endl;
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);
    icp_ptr_ -> setInputTarget(input_target);
    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    icp_ptr_ -> setInputSource(input_source);

    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();
    float ndtScore = ndt_ptr_ -> getFitnessScore();
    std::cout << "NDTScore :" << ndtScore << " --- ";

    icp_ptr_ -> align(*result_cloud_ptr, result_pose);
    result_pose = icp_ptr_ -> getFinalTransformation();
    float icpScore = icp_ptr_ -> getFitnessScore();
    std::cout << "ICPScore :" << icpScore << "  " << std::endl;
    std::cout << "------------------------------------------" << std::endl;

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}
}
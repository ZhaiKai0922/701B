/*
  *@Author: Zhai Kai
  *@Data:2021-10-28 09:39
*/
#include <deque>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/make_shared.hpp>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_representation.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/filter.h>
#include <pcl-1.8/pcl/features/normal_3d.h> //法线特征相关头文件

#include <pcl-1.8/pcl/registration/icp.h>  //ICP类相关头文件
#include <pcl-1.8/pcl/registration/icp_nl.h> //非线性ICP类相关头文件
#include <pcl-1.8/pcl/registration/ndt.h>

#include <pcl-1.8/pcl/registration/transforms.h>  //变换矩阵类相关头文件
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h> //可视化类相关文件
#include <glog/logging.h>
#include "../include/cloud_data.hpp"
#include "../include/registration_interface.hpp"
#include "../include/icp_registration.hpp"

#include <time.h>

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
using PointNormalT = pcl::PointNormal;
using PointCloudWithNormals = pcl::PointCloud<PointNormalT>;

//ICP 参数设置：
std::shared_ptr<pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>> ndt_ptr_ 
= std::make_shared<pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>>();
std::shared_ptr<pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>> icp_ptr_ 
= std::make_shared<pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>>();

pcl::visualization::PCLVisualizer *p; //创建可视化对象
int vp_1, vp_2;

class key_frame{
    public:
    key_frame():frame_ptr(new CloudData::CLOUD()){ };
    
    public:
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    CloudData::CLOUD_PTR frame_ptr;
    std::string f_name;
};

double getmse(CloudData::CLOUD_PTR modelPointCloud, CloudData::CLOUD_PTR queryPointCloud) {
	pcl::KdTreeFLANN <CloudData::POINT, flann::L2_Simple<float> > modelTree;
	modelTree.setInputCloud(modelPointCloud->makeShared());
	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);
	double fitness_score = 0.0;
	int count = 0;
	//For each point in the source PointCloud
	for (int i = 0; i < queryPointCloud->points.size(); i++) {
		//Find nearest neighbor in the target (query)
		modelTree.nearestKSearch(queryPointCloud->points[i], 1, nn_indices, nn_dists);
		count++;
		fitness_score += nn_dists[0];
	}
	if (count > 0)
		return (fitness_score / count);
	else
		return (std::numeric_limits<double >::max());
}

void showCloudLeft(const CloudData::CLOUD_PTR cloud_target, const CloudData::CLOUD_PTR cloud_source)
{
    p -> removePointCloud("vp1_target");
    p -> removePointCloud("vp1_source");
    PointCloudColorHandlerCustom<CloudData::POINT>  tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<CloudData::POINT>  src_h (cloud_source, 255, 0, 0);
    p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);
    PCL_INFO ("Press q to begin the registration.\n");
    //p-> spin();
}

void showCloudRight(const CloudData::CLOUD_PTR cloud_target, const CloudData::CLOUD_PTR cloud_source)
{
    p -> removePointCloud("source");
    p -> removePointCloud("target");
    PointCloudColorHandlerCustom<CloudData::POINT>  tgt_h_two (cloud_target, 0, 255, 0); //绿色
    PointCloudColorHandlerCustom<CloudData::POINT>  src_h_two (cloud_source, 255, 0, 0);

    p->addPointCloud (cloud_target, tgt_h_two, "target", vp_2);
    p->addPointCloud (cloud_source, src_h_two, "source", vp_2);
    //p -> spinOnce();
    //p-> spin();
}
// void showCloudRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
// {
//     p -> removePointCloud("source");
//     p -> removePointCloud("target");
//     PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler(cloud_target, "curvature");
//     PointCloudColorHandlerGenericField<PointNormalT> src_color_handler(cloud_source, "curvature");

//     p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
//     p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);
//     p -> spinOnce();
// }


void loadData(int argc, char** argv, std::deque<key_frame>& models)
{
    for(int i = 1; i < argc; i++)
    {
        std::string fname = std::string(argv[i]);

        key_frame temp;
        temp.f_name = fname;
        pcl::io::loadPCDFile(fname, *temp.frame_ptr);

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*temp.frame_ptr, *temp.frame_ptr, indices);
        LOG(INFO) << "Raw data size :" << temp.frame_ptr->size();
        key_frame KeyFrame;
        KeyFrame.f_name = fname;
        pcl::VoxelGrid<CloudData::POINT> voxel_grid;
        voxel_grid.setLeafSize(0.02, 0.02, 0.02);
        voxel_grid.setInputCloud(temp.frame_ptr);
        voxel_grid.filter(*KeyFrame.frame_ptr);
        LOG(INFO) << "Down size :" << KeyFrame.frame_ptr->size();
        models.push_back(KeyFrame);
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]); //初始化Glog库
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    //为了可视化目的，方便用户直观的观察到配准前后结果以及配准过程，
    //创建全局可视化对象变量，并定义左右视点分别显示配准前和配准后的结果点云

    std::deque<key_frame> data;
    loadData(argc, argv, data);

    if(data.empty())
    {
        LOG(WARNING) << "Data is empty!!!";
        return -1;
    }
    LOG(INFO) << "Data size :" << data.size();

    p = new pcl::visualization::PCLVisualizer(argc, argv, "Registration");
    p -> createViewPort(0.0, 0, 0.5, 1.0, vp_1);
    p -> createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_key_frame_pose = Eigen::Matrix4f::Identity();

    std::deque<key_frame> local_map_frames;
    CloudData::CLOUD_PTR local_map_ptr;
    CloudData::CLOUD_PTR result_ndt(new CloudData::CLOUD());
    CloudData::CLOUD_PTR result_icp(new CloudData::CLOUD());
    Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f final_pose = Eigen::Matrix4f::Identity();
    CloudData::CLOUD_PTR transformed_cloud(new CloudData::CLOUD());
    CloudData::CLOUD_PTR source, target;

    for(size_t i = 0; i < (data.size()-1); ++i)
    {
        local_map_frames.push_back(data[i]);
        local_map_ptr.reset(new CloudData::CLOUD());
        for(size_t j = 0; j < local_map_frames.size(); ++j)
        {
            pcl::transformPointCloud(*local_map_frames.at(i).frame_ptr,
                                                                    *transformed_cloud,
                                                                    local_map_frames.at(i).pose);
            *local_map_ptr += *transformed_cloud;
        }

        LOG(INFO) << "START TO REGISTRATION .....";

        ndt_ptr_ -> setResolution(0.8);
        ndt_ptr_ -> setStepSize(0.1); //0.1
        ndt_ptr_ -> setTransformationEpsilon(0.05);
        ndt_ptr_ -> setMaximumIterations(50);

        icp_ptr_->setMaxCorrespondenceDistance(1.2);  //1.2
        icp_ptr_->setTransformationEpsilon(0.01);
        icp_ptr_->setEuclideanFitnessEpsilon(0.36);
        icp_ptr_->setMaximumIterations(50);  //100

        // ndt_ptr_ -> setInputTarget(local_map_ptr);
        // LOG(INFO) << "Set NDT Input Target.....";
        // ndt_ptr_ -> setInputSource(data[i+1].frame_ptr);
        // LOG(INFO) << "Set NDT Input Source.....";

         double dur;
        // clock_t time_begin_ndt = clock();
        // ndt_ptr_ -> align(*result_ndt, predict_pose);  //???
        // clock_t time_end_ndt = clock();
        // dur = (double)(time_end_ndt - time_begin_ndt);

        // LOG(INFO) << "NDT align....."<<(dur/CLOCKS_PER_SEC) << "ms";
        
        // result_pose = ndt_ptr_ -> getFinalTransformation();

        // //pcl::transformPointCloud(*data[i+1].frame_ptr, *result_ndt, result_pose);
        // //***********************************************************************
        // double score_ndt = getmse(local_map_ptr, result_ndt);
        // LOG(INFO) << "NDT MSE Score:" << score_ndt;
        // LOG(INFO) << "Get Transformation";
        // LOG(INFO) << result_pose;

        icp_ptr_ -> setInputTarget(local_map_ptr);
        LOG(INFO) << "Set ICP Input Target.....";
        icp_ptr_ -> setInputSource(data[i+1].frame_ptr);
        LOG(INFO) << "Set ICP Input Source.......";

        clock_t time_begin_icp = clock();
        //icp_ptr_ -> align(*result_icp, result_pose);
        icp_ptr_ -> align(*result_icp, predict_pose);
        clock_t time_end_icp = clock();
        dur = (double)(time_end_icp - time_begin_icp);
        LOG(INFO) << "ICP align......"<<(dur/CLOCKS_PER_SEC) << "ms";

        final_pose = icp_ptr_ -> getFinalTransformation();

        //pcl::transformPointCloud(*result_ndt, *result_icp, final_pose);
        //pcl::transformPointCloud(*data[i+1].frame_ptr, *result_icp, final_pose);
        double score_icp = getmse(local_map_ptr, result_icp);
        LOG(INFO) << "ICP MSE Score:" << score_icp;

        LOG(INFO) << "Get Final Transformation";
        LOG(INFO) << final_pose;
        data[i+1].pose = final_pose;

        LOG(INFO) << "SHOW RIGHT .....";
        LOG(INFO) << "------------------------------------------------------------------";

        //Visual
        showCloudLeft(local_map_ptr, result_ndt);
        //Visual
        showCloudRight(local_map_ptr, result_icp);
        
        step_pose = last_pose.inverse() * result_pose;
        predict_pose = result_pose * step_pose;
        last_pose = result_pose;
    }

    p-> spin();
    return 0;

}
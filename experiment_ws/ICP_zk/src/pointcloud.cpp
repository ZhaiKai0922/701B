#include <iostream>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl-1.8/pcl/registration/transforms.h>  //变换矩阵类相关头文件

int main(int argc, char **argv) {
    //vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
   // vector<Eigen::Isometry3d> poses;         // 相机位姿

    // 定义点云使用的格式：这里用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    PointCloud::Ptr pointcloud(new PointCloud());

    vector<PointT> my_map;

    ifstream fin("pose.txt");
    if (!fin) {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    //**************************************
    for (int i = 1; i < 14; i++) {
        PointCloud::Ptr target_cloud(new PointCloud());
        boost::format fmt("%d.%s");
        //std::string fname = i + ".pcd";
        if (pcl::io::loadPCDFile<PointT>((fmt%(i+1)%"pcd").str(), *target_cloud) == -1)
        {
            PCL_ERROR("Couldn't read file room_scan1.pcd \n");
            return (-1);
        }
        cout << "Loaded " << target_cloud->size() << " data points from 1.pcd" << std::endl;

        double data[16] = {0};
        for (int i = 0; i < 16; i++) {
            fin >> data[i];
        }

        // Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
        // T1(0,0) = data[0], T1(0,1) = data[1], T1(0,2) = data[2], T1(0,3) = data[3];
        // T1(1,0) = data[4], T1(1,1) = data[5], T1(1,2) = data[6], T1(1,3) = data[7];
        // T1(2,0) = data[8], T1(2,1) = data[9], T1(2,2) = data[10], T1(2,3) = data[11];
        // T1(3,0) =  data[12], T1(3,1) = data[13], T1(3,2) = data[14], T1(3,3) =  data[15];

        Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
        T1(0,0) = data[0], T1(0,1) = data[1], T1(0,2) = data[2], T1(0,3) = data[3];
        T1(1,0) = data[4], T1(1,1) = data[5], T1(1,2) = data[6], T1(1,3) = data[7];
        T1(2,0) = data[8], T1(2,1) = data[9], T1(2,2) = data[10], T1(2,3) = data[11];
        T1(3,0) =  data[12], T1(3,1) = data[13], T1(3,2) = data[14], T1(3,3) =  data[15];

        //poses.push_back(T1);
        cout <<T1(0, 0) << T1(0, 1) << T1(0, 2) << T1(0, 3) << endl;
        cout <<T1(1, 0) << T1(1, 1) << T1(1, 2) << T1(1, 3) << endl;

        PointCloud::Ptr current(new PointCloud());

        pcl::transformPointCloud(*target_cloud, *current, T1);


        // for(int i = 0; i < target_cloud -> size(); i++)
        // {
        //     PointT m_p = target_cloud -> points[i];

        //     Eigen::Vector3d point;
        //     point[0] = m_p.x;
        //     point[1] = m_p.y;
        //     point[2] = m_p.z;
        //     Eigen::Vector3d pointWorld = T1 * point;

        //     m_p.x = pointWorld[0];
        //     m_p.y = pointWorld[1];
        //     m_p.z = pointWorld[2];
        
        //     current -> points.push_back(m_p);
        // }

        // PointCloud::Ptr tmp(new PointCloud);
        // pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        // statistical_filter.setMeanK(50);
        // statistical_filter.setStddevMulThresh(1.0);
        // statistical_filter.setInputCloud(current);
        // statistical_filter.filter(*tmp);

        (*pointcloud) += *current;
    }

    cout << "点云共有" << pointcloud->size() << "个点." << endl;

    //voxel filter
    // pcl::VoxelGrid<PointT> voxel_filter;
    // double resolution = 0.03;
    // voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
    // PointCloud::Ptr tmp(new PointCloud);
    // voxel_filter.setInputCloud(pointcloud);
    // voxel_filter.filter(*tmp);
    // tmp->swap(*pointcloud);

    //cout << "点云共有" << pointcloud->size() << "个点." << endl;

    pcl::io::savePCDFileBinary("map.pcd", *pointcloud);
    return 0;
}

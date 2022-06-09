#include <iostream>
#include <fstream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/filters/statistical_outlier_removal.h>

int main(int argc, char** argv)
{
    vector<cv::Mat> colorImgs, depthImgs;

    for(int i = 0; i < 21; i++)
    {
        boost::format fmt("../data_3/%s/%d.%s");   //图像文件格式
        colorImgs.push_back(cv::imread((fmt%"color"%(i + 1)%"png").str()));
        depthImgs.push_back(cv::imread((fmt%"depth"%(i + 1)%"png").str(), -1));  //使用-1读取原始图像
    }

    //定义点云使用的格式：这里使用的是XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

    // 相机内参 
    // double cx = 319.5;
    // double cy = 239.5;
    // double fx = 481.2;
    // double fy = -480.0;
    // double depthScale = 5000.0;

    // double cx = 160.5;
    // double cy = 120.5;
    // double fx = 277.19135641132203;
    // double fy = 277.19135641132203;
    // double depthScale = 5000.0;

    // double cx = 640.5;
    // double cy = 360.5;
    // double fx = 1108.7654256452881;
    // double fy = 1108.7654256452881;
    // double depthScale = 5000.0;

    // double cx = 363.03301985356075;
    // double cy = 243.75993682034533;
    // double fx = 362.1336172429237;
    // double fy = 383.3943871542167;
    // double depthScale = 5000.0;

    double cx = 256.8100891113281;
    double cy = 205.24139404296875;
    double fx = 364.1957092285156;
    double fy = 364.1957092285156;
    double depthScale = 5000.0;

    for(int i = 0; i < 21; i++){
        PointCloud::Ptr current(new PointCloud());
        cout << "Converting images ..." << i + 1<< endl;

        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];

        for(int v = 0; v < color.rows; v++)
            for(int u = 0; u < color.cols; u++)
            {
                unsigned int d = depth.ptr<unsigned short>(v)[u];  //深度值
                if(d == 0) continue;//为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                
                PointT p;
                p.x = point[0];
                p.y = point[1];
                p.z = point[2];
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];

                current->points.push_back(p);
            }
        
        PointCloud::Ptr tmp(new PointCloud());
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter(*tmp);

        //pointCloud->is_dense = false;
        cout << "PointCloud has" << tmp->size() << "points" << endl;

        pcl::VoxelGrid<PointT> voxel_filter;
        double resolution = 0.03;
        voxel_filter.setLeafSize(resolution, resolution, resolution);       // resolution
        PointCloud::Ptr tmp_2(new PointCloud());
        voxel_filter.setInputCloud(tmp);
        voxel_filter.filter(*tmp_2);

        cout << "滤波之后，点云共有" << tmp_2->size() << "个点." << endl;

        pcl::io::savePCDFileBinary(std::to_string(i + 1)+".pcd", *tmp_2);
    }

    return 0;

}

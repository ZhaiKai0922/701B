#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
int
main(int argc, char** argv)
{
    //加载房间的第一次扫描
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from 1.pcd" << std::endl;

    Eigen::Quaterniond q_target(0.999999, -0.00101358, 0.00052453, -0.000231475);
    Eigen::Isometry3d T_target(q_target);
    T_target.pretranslate(Eigen::Vector3d(0.000466347, 0.00895357, -2.24935));

    pcl::PointCloud<pcl::PointXYZ>::Ptr current(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < target_cloud -> size(); i++)
    {
        Eigen::Vector3d point;
        point[0] = target_cloud -> points[i].x;
        point[1] = target_cloud -> points[i].y;
        point[2] = target_cloud -> points[i].z;

        Eigen::Vector3d pointWorld = T_target * point;

        pcl::PointXYZ m_p;
        m_p.x = pointWorld[0];
        m_p.y = pointWorld[1];
        m_p.z = pointWorld[2];
        
        current -> points.push_back(m_p);

    }

    std::cout << "Current succeed !" << std::endl;


    //加载从新视角得到的房间的第二次扫描
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("3.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file room_scan2.pcd \n");
        return (-1);
    }

    //new
    Eigen::Quaterniond q_2(0.932926, 0.0492614, 0.323821, 0.14954);
    Eigen::Isometry3d T_2(q_2);
    T_2.pretranslate(Eigen::Vector3d(0.310932, -0.432757, -1.48048));

    pcl::PointCloud<pcl::PointXYZ>::Ptr current_2(new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i < input_cloud -> size(); i++)
    {
        Eigen::Vector3d point;
        point[0] = input_cloud -> points[i].x;
        point[1] = input_cloud -> points[i].y;
        point[2] = input_cloud -> points[i].z;

        Eigen::Vector3d pointWorld = T_2 * point;

        pcl::PointXYZ m_p;
        m_p.x = pointWorld[0];
        m_p.y = pointWorld[1];
        m_p.z = pointWorld[2];
        
        current_2 -> points.push_back(m_p);

    }
    
    std::cout << "Current_2 succeed !" << current_2->size() <<  std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    *pointCloud = *current + *current_2;
    pcl::io::savePCDFileBinary("pointCloud.pcd", *pointCloud);


    // //************************************************************
    // std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;
    // //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度。
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // approximate_voxel_filter.setInputCloud(input_cloud);
    // approximate_voxel_filter.filter(*filtered_cloud);
    // std::cout << "Filtered cloud contains " << filtered_cloud->size()
    //     << " data points from room_scan2.pcd" << std::endl;
    // //初始化正态分布变换（NDT）
    // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // //设置依赖尺度NDT参数
    // //为终止条件设置最小转换差异
    // ndt.setTransformationEpsilon(0.01);
    // //为More-Thuente线搜索设置最大步长
    // ndt.setStepSize(0.1);
    // //设置NDT网格结构的分辨率（VoxelGridCovariance）
    // ndt.setResolution(1.0);
    // //设置匹配迭代的最大次数
    // ndt.setMaximumIterations(100);
    // // 设置要配准的点云
    // ndt.setInputCloud(filtered_cloud);
    // //设置点云配准目标
    // ndt.setInputTarget(current);

    // //设置使用机器人测距法得到的初始对准估计结果
    // //Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    // Eigen::Quaternionf q(0.0492614, 0.323821, 0.14954, 0.932926);

    // Eigen::AngleAxisf init_rotation(q);

    // //Eigen::Isometry3d T(q);

    // //T.pretranslate(Eigen::Vector3f(0.323821, 0.14954, 0.932926));

    // //Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    // Eigen::Translation3f init_translation(0.310932, -0.432757, -1.48048);
    // Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();


    // //计算需要的刚体变换以便将输入的点云匹配到目标点云
    // pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // ndt.align(*output_cloud, init_guess);
    // //ndt.align(*output_cloud, T);
    // std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
    //     << " score: " << ndt.getFitnessScore() << std::endl;
    // //使用创建的变换对未过滤的输入点云进行变换
    // pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    // //*******************************************************


    //保存转换的输入点云
    //pcl::io::savePCDFileASCII("room_scan2_transformed.pcd", *output_cloud);
    // 初始化点云可视化界面
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    //对目标点云着色（红色）并可视化
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        //target_color(target_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color(current, 255, 0, 0);

    //viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->addPointCloud<pcl::PointXYZ>(current, target_color, "target cloud");


    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "target cloud");


    //对转换后的目标点云着色（绿色）并可视化
    //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        //output_color(output_cloud, 0, 255, 0);

     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        output_color(current_2, 0, 255, 0);

    //viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->addPointCloud<pcl::PointXYZ>(current_2, output_color, "output cloud");

    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        1, "output cloud");
    // 启动可视化
    viewer_final->addCoordinateSystem(1.0);
    viewer_final->initCameraParameters();
    //等待直到可视化窗口关闭。
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    return (0);
}


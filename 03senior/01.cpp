/*
 * 正态分布变换配准NDT
 */
#include<iostream>
#include<thread>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

#include<pcl/registration/ndt.h>//ndt标准头文件
#include<pcl/filters/approximate_voxel_grid.h>//滤波头文件
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
    //lading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../pcd/plate1.pcd", *target_cloud) == -1) {
        PCL_ERROR("cloudn,t read file room_scan\n");
        return -1;
    }
    std::cout << " Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;

    //loading second scan of room from new perspective
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../pcd/plate_temp_0.pcd", *input_cloud) == -1) {
        PCL_ERROR("cloudn,t read file room_scan\n");
        return -1;
    }
    std::cout << " Loaded " << input_cloud->size() << " data points form room_scan2.pcd" << std::endl;

    //体素滤波
    //过滤输入点云到10%的院士大小，目标点云target-cloud不需要进行滤波操作，因为NDT算法在目标点对应的体素网格数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;//体素网格过滤器
    approximate_voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filtered_cloud);
    std::cout << "filtered cloud contains " << filtered_cloud->size()<<  std::endl;

    //离散点滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);
    sor.setMeanK(5);    //设置平均距离估计的最近距离数量K
    sor.setStddevMulThresh(1.0);    //设置标准差异阀值系数
    sor.filter(*filtered_cloud);    //执行过滤

    //Initializing Normal Distributions transform
    //初始化正态分布变换
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置与尺度相关的NDT参数
    //设定终止条件的最小变换差值。
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(0.1);//为mor-thuente线搜索设置最大步长
    ndt.setResolution(1.0);//设置NDT网络结构分辨率
    ndt.setMaximumIterations(50);//设置匹配迭代最大次数
    ndt.setInputSource(filtered_cloud);//设置过滤后输入点云
    ndt.setInputTarget(target_cloud);//设置目标点云

    //设置使用机器人测距法得到的初始变换矩阵
    Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    //计算所需的刚体变换，保证输入云与目标云对齐
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);

//    std::cout << "normal distributions transform has converged" << ndt.hasConverged()
//              << " score: " << ndt.getFitnessScore() << std::endl;

    //transforming unfiltered input cloud using found transform
    //使用找到的变换矩阵，来对未过滤的输入云进行变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());

    //saving transformed input cloud
    //保存变换后的输入云
    pcl::io::savePCDFile("../transformed.pcd", *output_cloud);

    //initializing point cloud visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer_final(new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);

    //coloring and visualizing traget cloud(red)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 200, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    //coloring and visualizing transformed input input cloud(green)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(target_cloud, 0, 200, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

    //starting visualizer
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    //wait until visualizer window is closed
    while (!viewer_final->wasStopped()) {
        viewer_final->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return 0;
}
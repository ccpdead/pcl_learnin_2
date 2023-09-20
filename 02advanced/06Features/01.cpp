/*
 * 法向量估计实现（1）
 */
#include<pcl/visualization/cloud_viewer.h>
#include<iostream>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include<pcl/features/normal_3d.h>

int main()
{
    //load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("../../../data/c1.pcd",*cloud);
    //estimate normal
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    //object for normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.05);
    //创建一个空的kdtree，讲只传递给法向量估计对象
    //这个tree对象将会在ne内部根据输入数据进行填充
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(kdtree);
    ne.compute(*normals);

    //visualize normals
    pcl::visualization::PCLVisualizer viewer("pcl viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // 参数int level=2 表示每n个点绘制一个法向量
    // 参数float scale=0.01 表示法向量长度缩放为0.01倍
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals,2,0.01,"normals");
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
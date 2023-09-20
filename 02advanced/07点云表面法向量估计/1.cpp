/*
 * 估计点云法线
 */
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/cloud_viewer.h>

int main()
{
    //load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile("../../../data/person/person.pcd", *cloud);

    //创建一个法线估计的对象并计算法线
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(0.03);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(kdtree);
    ne.compute(*normals);

    //visualize normals
    pcl::visualization::PCLVisualizer viewer("pcl viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals);

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
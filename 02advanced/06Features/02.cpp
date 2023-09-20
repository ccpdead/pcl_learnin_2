/*
 * 法向量估计实现（2）
 */
#include <pcl/point_types.h>

#include<iostream>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include<pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

int main(int argc, char **argv) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    pcl::io::loadPCDFile("../../../data/c1.pcd", *cloud);

// 准备一个indices索引集合，为了简单起见，我们直接使用点云的前10%的点
    std::vector<int> indices(std::floor(cloud->points.size() / 10));
    for (std::size_t i = 0; i < indices.size(); ++i)
        indices[i] = i;

// 创建法向量估算类，设置输入点云
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

// 设置indices索引
    boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(indices));
    ne.setIndices(indicesptr);
// 创建一个空的kdtree，将值传递给法向量估算对象
// 这个tree对象将会在ne内部根据输入的数据集进行填充（这里设置没有其他的search surface）
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);// 使用一个半径为3cm的球体中的所有邻居点
    ne.compute(*cloud_normals);// 计算特征

//根据索引过滤点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(indicesptr);
    extract.setNegative(false);\
    extract.filter(*cloud_filed);

    //visualize normals
    pcl::visualization::PCLVisualizer viewer("pcl viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    // 参数int level=2 表示每n个点绘制一个法向量
    // 参数float scale=0.01 表示法向量长度缩放为0.01倍
    viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud_filed, cloud_normals, 2, 0.01, "normals");
    while (!viewer.wasStopped()) {
        viewer.spinOnce();
    }
}
/*
 * 离散点滤波
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //从文件读取点云
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("../pcd/capture0002.pcd", *cloud);
    
    std::cerr<<"Cloud before filterig:"<<std::endl;
    std::cerr<<*cloud<<std::endl;

    //创建过滤器，每个点分析计算时考虑最近邻居个数为50个；
    //设置标准差阀值为1，这意味着所有距离查询点的平均距离标准差均大于1个标准偏差的所有点将被标记为离群值并删除。
    //计算输出并将储存放在cloud-filtered中

    float leftSize = 0.01f;
    pcl::VoxelGrid<pcl::PointXYZ> vox;    //Create the filtering object
    vox.setInputCloud(cloud);
    vox.setLeafSize(leftSize, leftSize, leftSize);
    vox.filter(*cloud_filtered);


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);    //设置平均距离估计的最近距离数量K
    sor.setStddevMulThresh(1.0);    //设置标准差异阀值系数
    sor.filter(*cloud_filtered);    //执行过滤

    std::cerr<<"Cloud after filtering: "<<std::endl;
    std::cerr<<*cloud_filtered<<std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("../pcd/capture0002_inliers.pcd", *cloud_filtered, false);

    //使用相同的过滤器，但对输出结果取反，将被过滤的点另外储存。
    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    writer.write<pcl::PointXYZ>("../pcd/capture0002_outliers.pcd", *cloud_filtered, false);

    return 0;
}
//
// 通过直面分割，滤波等算法，剥离单独的盘子等点云数据
//
/**
 * 直通滤波器
* 体素滤波
 * 邻近点个数滤波
 * 平滑数据
*/
#include "iostream"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include <pcl/filters/passthrough.h>//直通滤波器
#include "pcl/filters/voxel_grid.h"//体素滤波器
#include "pcl/filters/radius_outlier_removal.h"//邻近点滤波器
#include <pcl/kdtree/kdtree_flann.h>//kdtree搜索
#include <pcl/surface/mls.h>//平滑滤波器


#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv){

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read("../plate1.pcd",*cloud);

    std::cerr<<"Pointcloud before filtering: "<<cloud->width*cloud->height
             <<" data points("<<pcl::getFieldsList(*cloud)<<")."<<std::endl;
    //直通滤波器
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.2, 0.2);
    pass.filter(*cloud);

    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-0.2, 0.2);
    pass.filter(*cloud);

    //体素滤波器
    pcl::VoxelGrid<pcl::PointXYZ>sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.002f,0.002f,0.002f);
    sor.filter(*cloud_filtered);

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals (true);

    // Set parameters
//    mls.setInputCloud (cloud_filtered);
//    mls.setPolynomialOrder (true);
//    mls.setSearchMethod (tree);
//    mls.setSearchRadius (0.03);
//    mls.process (*mls_points);

    std::cerr<<"Pointcloud before filtering: "<<cloud_filtered->width*cloud_filtered->height
             <<" data points("<<pcl::getFieldsList(*cloud_filtered)<<")."<<std::endl;

//    pcl::visualization::CloudViewer viewer("viewer");
//    viewer.showCloud(mls_points);
//    while(!viewer.wasStopped()){
//
//
//    }
    pcl::io::savePCDFile("../plate1_temp.pcd",*cloud_filtered);
    return 0;
}
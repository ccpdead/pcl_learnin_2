#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

int
main (int argc, char** argv)
{
    pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_p (new pcl::PointCloud<pcl::PointXYZ>),
            cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("../plate1_temp.pcd", *cloud_blob);

//    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;//体素滤波器
//    sor.setInputCloud (cloud_blob);
//    sor.setLeafSize (0.002f, 0.002f, 0.002f);
//    sor.filter (*cloud_filtered_blob);

    // Convert to the templated PointCloud
    pcl::fromPCLPointCloud2 (*cloud_blob, *cloud_filtered);

//    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;//保存降采样后的数据
//    writer.write<pcl::PointXYZ> ("../plate2_mls_downsampled.pcd", *cloud_filtered, false);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());//点云索引
    // Create the segmentation object 创建分割器对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);//平面检测器
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.002);

    // Create the filtering object
    //通过ExtractIndices滤波器提取点云中的子集
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.8 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        //从剩余的云中分割出最大的平面组件
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        std::stringstream ss;//字符串与整形转换
        ss << "../plate_temp_" << i << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

        // Create the filtering object
        extract.setNegative (false);
        extract.filter (*cloud_f);
        writer.write<pcl::PointXYZ>("../plate_temp_p.pcd",*cloud_f,false);
        cloud_filtered.swap (cloud_f);

        i++;
    }

    return (0);
}

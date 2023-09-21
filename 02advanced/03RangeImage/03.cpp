//
// Created by jhr on 9/21/23.
//
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/range_image/range_image.h>

int main(int argc, char**argv){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (float x = -1.0; x <= 1.0; x += 0.01) {
        for (float y = -1.0; y <= 1.0; y += 0.01) {
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = std::sin(x * y);
            cloud->push_back(point);
        }
    }

    boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage &rangeImage = *range_image_ptr;
    rangeImage.createFromPointCloud(*cloud, 1.0, pcl::deg2rad(0.5), pcl::deg2rad(360.0), 0, 0, 0);
    pcl::visualization::CloudViewer viewer("Depth Image Viewer");
    viewer.showMonoImage(rangeImage);
    while (!viewer.wasStopped()) {
        // 等待查看器窗口关闭
    }



}


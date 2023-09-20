1,通过cloudviewer显示点云
```c++
    pcl::visualization::CloudViewer viewer ("test");
    viewer.showCloud(point_cloud_ptr);
    while (!viewer.wasStopped()){ };
```
2,通过pcl::visualize显示点云
```c++
    boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0.05,0.05,0.05,0);
    viewer->addPointCloud<pcl::PointXYZRGB>(point_cloud_ptr,"sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,5,"sample cloud");\

```
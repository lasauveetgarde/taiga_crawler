void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  float depthThreshold = 0.5;
  float threshold2 = depthThreshold*depthThreshold;

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  for (int p=0; p<cloud->points.size(); ++p)
  {
    // find the squared distance from the origin.
    float pointDepth2 = (cloud->points[p].x * cloud->points[p].x) +
                       (cloud->points[p].y * cloud->points[p].y) + 
                       (cloud->points[p].z * cloud->points[p].z));

    // remove point if it's within the threshold range
    if (pointDepth2 < threshold2)
    {
      cloud->points[p] = cloud->points[cloud->points.size()-1];
      cloud->points.resize(cloud->points.size()-1);
      --p;
    }
  }

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.1, 0.1, 0.1);
  sor.filter (cloud_filtered);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish (output);
}
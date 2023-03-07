#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Create a point cloud object to store the laser scan data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the point cloud data from the laser scan message
    for (int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float range = scan_msg->ranges[i];
        if (std::isnan(range) || range < scan_msg->range_min || range > scan_msg->range_max)
        {
            continue;
        }
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        pcl::PointXYZ point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.z = 0;
        cloud->points.push_back(point);
    }

    // Downsample the point cloud using voxel grid filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(0.05, 0.05, 0.05);
    voxel_grid.filter(*downsampled_cloud);

    // Perform Euclidean cluster extraction on the downsampled point cloud
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extraction;
    cluster_extraction.setInputCloud(downsampled_cloud);
    cluster_extraction.setClusterTolerance(0.2);
    cluster_extraction.setMinClusterSize(10);
    cluster_extraction.setMaxClusterSize(10000);
    cluster_extraction.extract(cluster_indices);

    // Create a point cloud object for each cluster and publish it
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            cluster_cloud->points.push_back(downsampled_cloud->points[*pit]);
        }
        cluster_cloud->width = cluster_cloud->points.size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cluster_cloud, cloud_msg);
        cloud_msg.header = scan_msg->header;
        pub.publish(cloud_msg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "euclidean_cluster_extraction");
    ros::NodeHandle nh;

    // Subscribe to the laser scan topic
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, laserScanCallback);

    // Advertise the point cloud topic
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    ros::spin();

    return 0;
}

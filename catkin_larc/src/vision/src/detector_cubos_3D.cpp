#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <vision/objectDetectionArray.h>

// #include <gpd_ros/CloudSamples.h>
// #include <gpd_ros/CloudSources.h>

// #include <actionlib/server/simple_action_server.h>
// #include <vision/DetectObjects3DAction.h>


#include <std_msgs/Int64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#define CAMERA_FRAME "head_rgbd_sensor_depth_frame"

using namespace octomap;

struct ObjectParams
{
  /* PointCloud Cluster. */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster;
  /* PointCloud Cluster Original. */
  pcl::PointCloud<pcl::PointXYZ> cluster_original;
  /* Mesh. */
  shape_msgs::Mesh::Ptr mesh;
  /* Center point of the Cluster. */
  geometry_msgs::PoseStamped center;
  /* Center point of the Cluster referenced to Camera TF. */
  geometry_msgs::PoseStamped center_cam;
  /* World Z Min Value. */
  double min_z;
  /* World Z Max Value. */
  double max_z;
  /* World X Min Value. */
  double min_x;
  /* World X Max Value. */
  double max_x;
  /* World Y Min Value. */
  double min_y;
  /* World Y Max Value. */
  double max_y;
  /* Detection Label. */
  int label = -1;
  /* Valid Object. */
  bool isValid = false;
  /* Valid Object. */
  int file_id = 0;
};

class CubeDetect3D
{
   const std::string name = "CubeDetect3D";
    ros::NodeHandle nh_;
    // actionlib::SimpleActionServer<object_detector::DetectObjects3DAction> as_;
    // object_detector::DetectObjects3DFeedback feedback_;
    // object_detector::DetectObjects3DResult result_;
    ros::Publisher pose_pub_;
    geometry_msgs::PoseArray pose_pub_msg_;
    ros::Publisher pc_pub_1_;
    ros::Publisher pc_pub_2_;
    ros::Publisher pc_pub_3_;
    ros::Subscriber sub_;
public: 
  CubeDetect3D() :
    listener_(buffer_)
    // as_(nh_, name, boost::bind(&Detect3D::handleActionServer, this, _1), false)
    {
      tf_listener = new tf::TransformListener();
      // as_.start();
      pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/test/objectposes", 10);
      pc_pub_1_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_1", 10);
      pc_pub_2_ = nh_.advertise<sensor_msgs::PointCloud2>("/test_pc_2", 10);
      pc_pub_3_ = nh_.advertise<sensor_msgs::Image>("/test_pc_3", 10);

      while(true && ros::ok())
      {
        sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", nh_));
        cloudCB(pc);
        // Wait 3 segs.
        ros::Duration(5).sleep();
      }

      // ROS_INFO_STREAM("Action Server Detect3D - Initialized");
    }
  //   /** \brief Handle Action Server Goal Received. */
  // void handleActionServer(const object_detector::DetectObjects3DGoalConstPtr &goal)
  // {
  //   ROS_INFO_STREAM("Action Server Detect3D - Goal Received");
  //   biggest_object_ = goal->force_object.detections.size() == 0;
  //   force_object_ = !biggest_object_ ? goal->force_object.detections[0] : object_detector::objectDetection();
  //   feedback_.status = 0;
  //   // result_.object_cloud = gpd_ros::CloudSamples();
  //   result_.object_pose = geometry_msgs::PoseStamped();
  //   result_.x_plane = 0;
  //   result_.y_plane = 0;
  //   result_.z_plane = 0;
  //   result_.width_plane = 0;
  //   result_.height_plane = 0;
  //   pose_pub_msg_.poses.clear();

  //   // gpd_msg_.cloud_sources = gpd_ros::CloudSources();
  //   // gpd_msg_.samples.clear();

  //   if (as_.isPreemptRequested() || !ros::ok())
  //   {
  //     ROS_INFO_STREAM("Action Server Detect3D - Preempted");
  //     as_.setPreempted(); // Set the action state to preempted
  //     return;
  //   }

  //   // Get PointCloud and Transform it to Map Frame
  //   sensor_msgs::PointCloud2 pc;
  //   sensor_msgs::PointCloud2 t_pc;
  //   tf_listener->waitForTransform("/map", CAMERA_FRAME, ros::Time(0), ros::Duration(5.0));
  //   pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/cloud_registered", nh_));
  //   pc.header.frame_id = CAMERA_FRAME;
  //   pcl_ros::transformPointCloud("map", pc, t_pc, *tf_listener);

  //   Detect3D::cloudCB(t_pc);
  //   as_.setSucceeded(result_);
  //   pose_pub_.publish(pose_pub_msg_);
  // }

 // /** \brief Given a pointcloud extract the ROI defined by the user.
  // @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    // min and max values in z axis to keep
    pass.setFilterLimits(0, 5.0);
    pass.filter(*cloud);
  }

  /** \brief Given the pointcloud and pointer cloud_normals compute the point normals and store in cloud_normals.
    @param cloud - Pointcloud.
    @param cloud_normals - The point normals once computer will be stored in this. */
  void computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    // Set the number of k nearest neighbors to use for the feature estimation.
    ne.setKSearch(50);
    ne.compute(*cloud_normals);
  }

  /** \brief Given the point normals and point indices, extract the normals for the indices.
      @param cloud_normals - Point normals.
      @param inliers - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers);
    extract_normals.filter(*cloud_normals);
  }
  
   /** \brief Given the pointcloud containing just the object,
      compute its center point, its height and its mesh and store in object_found.
      @param cloud - point cloud containing just the object. */
  void extractObjectDetails(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, ObjectParams& object_found, ObjectParams &table_params, bool isCluster = true)
  {
    ROS_INFO_STREAM("Extracting Object Details " << cloud->points.size());
    double max_z_ = cloud->points[0].z;
    double min_z_ = cloud->points[0].z;
    double max_y_ = cloud->points[0].y;
    double min_y_ = cloud->points[0].y;
    double max_x_ = cloud->points[0].x;
    double min_x_ = cloud->points[0].x;
    for (auto const point : cloud->points)
    {
      max_z_ = fmax(max_z_, point.z);
      min_z_ = fmin(min_z_, point.z);
      max_x_ = fmax(max_x_, point.x);
      min_x_ = fmin(min_x_, point.x);
      max_y_ = fmax(max_y_, point.y);
      min_y_ = fmin(min_y_, point.y);
    }
    object_found.max_z = max_z_;
    object_found.min_z = min_z_;
    object_found.max_x = max_x_;
    object_found.min_x = min_x_;
    object_found.max_y = max_y_;
    object_found.min_y = min_y_;

    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);

    // Store the object centroid.
    object_found.center.header.frame_id = "map";
    object_found.center.pose.position.x = centroid.x;
    object_found.center.pose.position.y = centroid.y;
    object_found.center.pose.position.z = centroid.z;
    object_found.center.pose.orientation.x = 0.0;
    object_found.center.pose.orientation.y = 0.0;
    object_found.center.pose.orientation.z = 0.0;
    object_found.center.pose.orientation.w = 1.0;

    // Store the object centroid referenced to the camera frame.
    geometry_msgs::TransformStamped tf_to_cam;
    tf_to_cam = buffer_.lookupTransform(CAMERA_FRAME, "map", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(object_found.center, object_found.center_cam, tf_to_cam);

    if (isCluster) {
      object_found.isValid = true;
      // Add object only if is above table.
      if (object_found.center.pose.position.z < table_params.min_z - 0.025) {
        ROS_INFO_STREAM("Object rejected due to table height: "
              << " x: " << object_found.center.pose.position.x
              << " y: " << object_found.center.pose.position.y
              << " z: " << object_found.center.pose.position.z
              << " Tz: " << table_params.min_z);
        object_found.isValid = false;
        return;
      }

      ROS_INFO_STREAM("Object dimensions: "
        << " " << abs(object_found.max_x - object_found.min_x)
        << " " << abs(object_found.max_y - object_found.min_y)
        << " " << abs(object_found.max_z - object_found.min_z));

      // Add object only if it has restricted dimensions.
      if (abs(object_found.max_x - object_found.min_x) > 0.5 ||
        abs(object_found.max_y - object_found.min_y) > 0.5 ||
        abs(object_found.max_z - object_found.min_z) > 0.5) {
        ROS_INFO_STREAM("Object rejected due to dimensions.");
        object_found.isValid = false;
        return;
      }

      object_found.cluster_original = *cloud;
      // Change point cloud origin to object centroid and save it.
      for (auto &point : cloud->points)
      {
        point.x = point.x - centroid.x;
        point.y = point.y - centroid.y;
        point.z = point.z - centroid.z;
      }
      object_found.cluster = cloud;
    }
  }

   /** \brief Given the pointcloud remove the biggest plane from the pointcloud. Return True if its the table.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. (output) */
      
  bool removeBiggestPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane, ObjectParams &plane_params)
  {
    // Do Plane Segmentation
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    segmentor.setMaxIterations(1000); /* run at max 1000 iterations before giving up */
    segmentor.setDistanceThreshold(0.01); /* tolerance for variation from model */
    segmentor.setInputCloud(cloud);
    
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    
    /* Extract the planar inliers from the input cloud save them in planeCloud */
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(false);
    extract_indices.filter(*planeCloud);

    /* Extract the planar inliers from the input cloud */
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);

    pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
    pcl::io::savePCDFile("pcl_no_table.pcd", *cloud);

    return cloud->size() > 0;
  }

  /** \brief PointCloud callback. */
  void cloudCB(const sensor_msgs::PointCloud2& input)
  {
    // Get cloud ready
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices); // Indices that correspond to a plane.
    pcl::fromROSMsg(input, *cloud);
    pcl::fromROSMsg(input, *cloud_original);

    sensor_msgs::PointCloud2 tmp;
    pcl::toROSMsg(*cloud, tmp);
    tmp.header.frame_id = input.header.frame_id;
    tmp.header.stamp = input.header.stamp;
    pc_pub_2_.publish(tmp);


    passThroughFilter(cloud);


    pcl::toROSMsg(*cloud, tmp);
    tmp.header.frame_id = input.header.frame_id;
    tmp.header.stamp = input.header.stamp;
    pc_pub_1_.publish(tmp);


    computeNormals(cloud, cloud_normals);

    // Detect and Remove the table on which the object is resting.
    ObjectParams table_params;
    removeBiggestPlane(cloud, inliers_plane, table_params);
    extractNormals(cloud_normals, inliers_plane);

    if (cloud->points.empty()) {
      return;
    }

    /* Extract all objects from PointCloud using Clustering. */
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    getClusters(cloud, clusters);
    int clustersFound = clusters.size();

    ROS_INFO_STREAM("Clusters Found: " << clustersFound);

    if (clustersFound == 0) {
      return;
    }

    // Convert PointCloud2 message to PointCloud object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(input, *cloud_rgb);

    for (int a = 0; a < clustersFound; a++) {
      // Extract RGB values and create sensor_msgs Image
      sensor_msgs::Image image_msg;
      image_msg.header = input->header;
      image_msg.height = cloud_rgb->height;
      image_msg.width = cloud_rgb->width;
      image_msg.encoding = "rgb8";
      image_msg.is_bigendian = false;
      image_msg.step = cloud_rgb->width * 3;
      size_t data_size = cloud_rgb->width * cloud_rgb->height * 3;
      image_msg.data.resize(data_size);

      int k = 0;
      for(int i = 0; i < cloud_rgb->height; i++)
      {
          for(int j = 0; j < cloud_rgb->width; j++)
          {
              image_msg.data[k++] = cloud_rgb->points[k].r;
              image_msg.data[k++] = cloud_rgb->points[k].g;
              image_msg.data[k++] = cloud_rgb->points[k].b;
          }
      }

      pc_pub_3_.publish(image_msg);
    }

    // Save all clusters to a file.
    for (int i = 0; i < clustersFound; i++) {
      std::stringstream ss;
      ss << "pcl_cluster_" << i << ".pcd";
      pcl::io::savePCDFile(ss.str(), *clusters[i]);
    }
  }

    /** \brief Find all clusters in a pointcloud.*/
  void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) 
  {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.025); // 2.5cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(30000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*

      // Discard Noise
      if (cloud_cluster->points.size() < 1000){
        continue; 
      }
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;

      clusters.push_back(cloud_cluster);
    }
  }

private:
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf::TransformListener *tf_listener;

};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "detect_objects_and_table");

  // Start the segmentor
   CubeDetect3D segmentor;
  

  // Spin
  ros::spin();
}


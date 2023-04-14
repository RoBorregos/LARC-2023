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

#include <object_detector/objectDetectionArray.h>

#include <gpd_ros/CloudSamples.h>
#include <gpd_ros/CloudSources.h>

#include <actionlib/server/simple_action_server.h>
#include <object_detector/DetectObjects3DAction.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>


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
  const std::string name "CubeDetect3D"
  ros::NodeHandle nh_;
  object_detector::DetectObjects3DFeedback feedback_;
  object_detector::DetectObjects3DResult result_;
  object_detector::objectDetection force_object_;

  //publishers  
  ros::Publisher pose_pub_;

  //subscribers
  ros::Subscriber sub;
  ros::Subscriber subscriberDepth;
  ros::Subscriber subscriberInfo;
  ros::Subscriber subscriberCloud;

  geometry_msgs::PoseArray pose_pub_msg_;
  gpd_ros::CloudSamples gpd_msg_
  ros::ServiceClient clear_octomap;

  public:
    CubeDetect3D() : 
      listener_(tfBuffer_), 
      as_(nh_, name, boost::bind(&Detect3D::handleActionServer, this, _1), false)
    {
        as_.start();
        ROS_INFO("Waiting for Clear Octomap service to start.");

        pc_pub_1_ = nh_.subscribe<message_package::Message>("/zed2/zed_node/point_cloud/cloud_registered", 1000, callbackFunction);

    }

    bool addPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::ModelCoefficients::Ptr& coefficients_plane, ObjectParams &plane_params)
    {
      extractObjectDetails(cloud, plane_params, plane_params, false);

      // Z conditions to know if it is the Table.
      if (plane_params.max_z - plane_params.min_z > 0.1) 
      { // Diff > 10cm, Not Parallel Plane 
        return false;
      }
      if (plane_params.min_z < 0.20) 
      { // Z Value < than 20cm, Floor Detected.
        return false;
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

      bool isTheTable = false;
      isTheTable = addPlane(planeCloud, coefficients_plane, plane_params);
      if (isTheTable) {
        pcl::io::savePCDFile("pcl_table.pcd", *planeCloud);
        pcl::io::savePCDFile("pcl_no_table.pcd", *cloud);
      }
      return isTheTable;
    }
  
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "detect_objects_and_table");

  // Start the segmentor
  Detect3D segmentor;

  // Spin
  ros::spin();
}


#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
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


#include <pcl/filters/project_inliers.h>
#include <vision/objectDetectionArray.h>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// #include <gpd_ros/CloudSamples.h>
// #include <gpd_ros/CloudSources.h>

// #include <actionlib/server/simple_action_server.h>
// #include <vision/DetectObjects3DAction.h>


#include <std_msgs/Int64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <limits>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>



#define CAMERA_FRAME "zed2_left_camera_frame"

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

struct Intrinsics
{
  int width;
  int height;
  double ppx;
  double ppy;
  double fx;
  double fy;
  std::string model;
  std::vector<double> coeffs;
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
    ros::Publisher pc_pub_img;
    ros::Publisher point_pub1;
    ros::Publisher point_pub2;
    sensor_msgs::CameraInfo zed_info_;
    sensor_msgs::Image zed_image_;
    cv_bridge::CvImagePtr zed_image_cv;

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

      pc_pub_img = nh_.advertise<sensor_msgs::Image>("/object", 10);


      zed_info_ = *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/zed2/zed_node/depth/camera_info", nh_));
      zed_image_ = *(ros::topic::waitForMessage<sensor_msgs::Image>("/zed2/zed_node/rgb/image_rect_color", nh_));
      zed_image_cv = cv_bridge::toCvCopy(zed_image_, sensor_msgs::image_encodings::BGR8);

      point_pub1 = nh_.advertise<geometry_msgs::PointStamped>("/point1", 10);
      point_pub2 = nh_.advertise<geometry_msgs::PointStamped>("/point2", 10);

      while(true && ros::ok())
      {
        // zed2_left_camera_frame
        sensor_msgs::PointCloud2 pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/zed2/zed_node/point_cloud/ds_cloud_registered", nh_));
        sensor_msgs::PointCloud2 t_pc;
        tf_listener->waitForTransform("/map", CAMERA_FRAME, ros::Time(0), ros::Duration(5.0));
        pc.header.frame_id = CAMERA_FRAME;
        pcl_ros::transformPointCloud("map", pc, t_pc, *tf_listener);

        cloudCB(t_pc);
        // Wait 3 segs.
        ros::Duration(3).sleep();
      }

      // ROS_INFO_STREAM("Action Server Detect3D - Initialized");
    }
  //   /** \brief Handle Action Server Goal Received. */
  // void handleActionServer(const objects_detector::DetectObjects3DGoalConstPtr &goal)
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
  
    void project_point_to_pixel(float pixel[2], float point[3], sensor_msgs::CameraInfo cam_info)
  {
    Intrinsics intrin;
    intrin.width = cam_info.width;
    intrin.height = cam_info.height;
    intrin.ppx = cam_info.K[2];
    intrin.ppy = cam_info.K[5];
    intrin.fx = cam_info.K[0];
    intrin.fy = cam_info.K[4];
    for (int i = 0; i < 5; i++) {
      intrin.coeffs.push_back(cam_info.D[i]);
    }
    if(cam_info.distortion_model == "plumb_bob") {
      intrin.model = "RS2_DISTORTION_BROWN_CONRADY";
    } else if(cam_info.distortion_model == "equidistant") {
      intrin.model = "RS2_DISTORTION_KANNALA_BRANDT4";
    }
        
    float x = point[0] / point[2], y = point[1] / point[2];

    if ((intrin.model == "RS2_DISTORTION_MODIFIED_BROWN_CONRADY") ||
        (intrin.model == "RS2_DISTORTION_INVERSE_BROWN_CONRADY"))
    {

        float r2 = x * x + y * y;
        float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
        x *= f;
        y *= f;
        float dx = x + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
        float dy = y + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);
        x = dx;
        y = dy;
    }

    if (intrin.model == "RS2_DISTORTION_BROWN_CONRADY")
    {
        float r2 = x * x + y * y;
        float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;

        float xf = x * f;
        float yf = y * f;

        float dx = xf + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
        float dy = yf + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);

        x = dx;
        y = dy;
    }

    if (intrin.model == "RS2_DISTORTION_FTHETA")
    {
        float r = sqrtf(x * x + y * y);
        if (r < FLT_EPSILON)
        {
            r = FLT_EPSILON;
        }
        float rd = (float)(1.0f / intrin.coeffs[0] * atan(2 * r * tan(intrin.coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
    }
    if (intrin.model == "RS2_DISTORTION_KANNALA_BRANDT4")
    {
        float r = sqrtf(x * x + y * y);
        if (r < FLT_EPSILON)
        {
            r = FLT_EPSILON;
        }
        float theta = atan(r);
        float theta2 = theta * theta;
        float series = 1 + theta2 * (intrin.coeffs[0] + theta2 * (intrin.coeffs[1] + theta2 * (intrin.coeffs[2] + theta2 * intrin.coeffs[3])));
        float rd = theta * series;
        x *= rd / r;
        y *= rd / r;
    }

    pixel[0] = x * intrin.fx + intrin.ppx;
    pixel[1] = y * intrin.fy + intrin.ppy;
  }

  void project_pixel_to_point(float point[3], float pixel[2], float depth, sensor_msgs::CameraInfo cam_info)
  {
    Intrinsics intrin;
    intrin.width = cam_info.width;
    intrin.height = cam_info.height;
    intrin.ppx = cam_info.K[2];
    intrin.ppy = cam_info.K[5];
    intrin.fx = cam_info.K[0];
    intrin.fy = cam_info.K[4];
    for (int i = 0; i < 5; i++) {
      intrin.coeffs.push_back(cam_info.D[i]);
    }
    if(cam_info.distortion_model == "plumb_bob") {
      intrin.model = "RS2_DISTORTION_BROWN_CONRADY";
    } else if(cam_info.distortion_model == "equidistant") {
      intrin.model = "RS2_DISTORTION_KANNALA_BRANDT4";
    }

    float x = (pixel[0] - intrin.ppx) / intrin.fx;
    float y = (pixel[1] - intrin.ppy) / intrin.fy;

    float xo = x;
    float yo = y;

    if (intrin.model == "RS2_DISTORTION_INVERSE_BROWN_CONRADY")
    {
        // need to loop until convergence 
        // 10 iterations determined empirically
        for (int i = 0; i < 10; i++)
        {
            float r2 = x * x + y * y;
            float icdist = (float)1 / (float)(1 + ((intrin.coeffs[4] * r2 + intrin.coeffs[1]) * r2 + intrin.coeffs[0]) * r2);
            float xq = x / icdist;
            float yq = y / icdist;
            float delta_x = 2 * intrin.coeffs[2] * xq * yq + intrin.coeffs[3] * (r2 + 2 * xq * xq);
            float delta_y = 2 * intrin.coeffs[3] * xq * yq + intrin.coeffs[2] * (r2 + 2 * yq * yq);
            x = (xo - delta_x) * icdist;
            y = (yo - delta_y) * icdist;
        }
    }
    if (intrin.model == "RS2_DISTORTION_BROWN_CONRADY")
    {
        // need to loop until convergence 
        // 10 iterations determined empirically
        for (int i = 0; i < 10; i++)
        {
            float r2 = x * x + y * y;
            float icdist = (float)1 / (float)(1 + ((intrin.coeffs[4] * r2 + intrin.coeffs[1]) * r2 + intrin.coeffs[0]) * r2);
            float delta_x = 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
            float delta_y = 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);
            x = (xo - delta_x) * icdist;
            y = (yo - delta_y) * icdist;
        }

    }
    if (intrin.model == "RS2_DISTORTION_KANNALA_BRANDT4")
    {
        float rd = sqrtf(x * x + y * y);
        if (rd < FLT_EPSILON)
        {
            rd = FLT_EPSILON;
        }

        float theta = rd;
        float theta2 = rd * rd;
        for (int i = 0; i < 4; i++)
        {
            float f = theta * (1 + theta2 * (intrin.coeffs[0] + theta2 * (intrin.coeffs[1] + theta2 * (intrin.coeffs[2] + theta2 * intrin.coeffs[3])))) - rd;
            if (fabs(f) < FLT_EPSILON)
            {
                break;
            }
            float df = 1 + theta2 * (3 * intrin.coeffs[0] + theta2 * (5 * intrin.coeffs[1] + theta2 * (7 * intrin.coeffs[2] + 9 * theta2 * intrin.coeffs[3])));
            theta -= f / df;
            theta2 = theta * theta;
        }
        float r = tan(theta);
        x *= r / rd;
        y *= r / rd;
    }
    if (intrin.model == "RS2_DISTORTION_FTHETA")
    {
        float rd = sqrtf(x * x + y * y);
        if (rd < FLT_EPSILON)
        {
            rd = FLT_EPSILON;
        }
        float r = (float)(tan(intrin.coeffs[0] * rd) / atan(2 * tan(intrin.coeffs[0] / 2.0f)));
        x *= r / rd;
        y *= r / rd;
    }

    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
  
  }


  void cropImage(cv::Mat &img, ObjectParams obj) {
    ROS_INFO_STREAM("---------------------Entreeeee a crop---------------------");
    float margin = 0.05; // 5cm
    // Get Points and Project Points To Pixel
    float pixel_left_up[2];
    float point_left_up[3];
    point_left_up[0] = obj.max_x + margin;
    point_left_up[1] = obj.max_y + margin;
    point_left_up[2] = obj.max_z + margin;
    // Transform point to camera frame
    tf::StampedTransform transform;
    try {
      tf_listener->waitForTransform("zed2_left_camera_optical_frame", "map", ros::Time(0), ros::Duration(10.0));
      tf_listener->lookupTransform("zed2_left_camera_optical_frame", "map", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
    }
    tf::Vector3 point_lu(point_left_up[0], point_left_up[1], point_left_up[2]);
    tf::Vector3 point_transformed_lu = transform * point_lu;
    point_left_up[0] = point_transformed_lu.getX();
    point_left_up[1] = point_transformed_lu.getY();
    point_left_up[2] = point_transformed_lu.getZ();

    // Publish PointStamped
    geometry_msgs::PointStamped point_msg1;
    point_msg1.header.frame_id = "zed2_left_camera_optical_frame";
    point_msg1.header.stamp = ros::Time::now();
    point_msg1.point.x = point_left_up[0];
    point_msg1.point.y = point_left_up[1];
    point_msg1.point.z = point_left_up[2];
    point_pub1.publish(point_msg1);


    project_point_to_pixel(pixel_left_up, point_left_up, zed_info_);
    
    float pixel_right_down[2];
    float point_right_down[3];
    point_right_down[0] = obj.min_x - margin;
    point_right_down[1] = obj.max_y + margin;
    point_right_down[2] = obj.min_z - margin;
    // Transform point to camera frame
    tf::Vector3 point_rd(point_right_down[0], point_right_down[1], point_right_down[2]);
    tf::Vector3 point_transformed_rd = transform * point_rd;
    point_right_down[0] = point_transformed_rd.getX();
    point_right_down[1] = point_transformed_rd.getY();
    point_right_down[2] = point_transformed_rd.getZ();

    // Publish PointStamped
    geometry_msgs::PointStamped point_msg2;
    point_msg2.header.frame_id = "zed2_left_camera_optical_frame";
    point_msg2.header.stamp = ros::Time::now();
    point_msg2.point.x = point_right_down[0];
    point_msg2.point.y = point_right_down[1];
    point_msg2.point.z = point_right_down[2];
    point_pub2.publish(point_msg2);

    ROS_INFO_STREAM("---------------------Antes---------------------");

    project_point_to_pixel(pixel_right_down, point_right_down, zed_info_);
    ROS_INFO_STREAM("max_x: " << obj.max_x << " max_y: " << obj.max_y << " max_z: " << obj.max_z);
    ROS_INFO_STREAM("min_x: " << obj.min_x << " min_y: " << obj.min_y << " min_z: " << obj.min_z);
    ROS_INFO_STREAM("pixel_left_up: " << pixel_left_up[0] << " " << pixel_left_up[1]);
    ROS_INFO_STREAM("pixel_right_down: " << pixel_right_down[0] << " " << pixel_right_down[1]);

    // Check Limits
    pixel_left_up[0] = std::max(pixel_left_up[0], 0.0f);
    pixel_left_up[1] = std::max(pixel_left_up[1], 0.0f);
    pixel_left_up[0] = std::min(pixel_left_up[0], (float)img.rows);
    pixel_left_up[1] = std::min(pixel_left_up[1], (float)img.cols);
    pixel_right_down[0] = std::max(pixel_right_down[0], 0.0f);
    pixel_right_down[1] = std::max(pixel_right_down[1], 0.0f);
    pixel_right_down[0] = std::min(pixel_right_down[0], (float)img.rows);
    pixel_right_down[1] = std::min(pixel_right_down[1], (float)img.cols);

    ROS_INFO_STREAM("---------------------Despues---------------------");

    project_point_to_pixel(pixel_right_down, point_right_down, zed_info_);
    ROS_INFO_STREAM("max_x: " << obj.max_x << " max_y: " << obj.max_y << " max_z: " << obj.max_z);
    ROS_INFO_STREAM("min_x: " << obj.min_x << " min_y: " << obj.min_y << " min_z: " << obj.min_z);
    ROS_INFO_STREAM("pixel_left_up: " << pixel_left_up[0] << " " << pixel_left_up[1]);
    ROS_INFO_STREAM("pixel_right_down: " << pixel_right_down[0] << " " << pixel_right_down[1]);

    // Crop rect && Save Image
    int x = pixel_left_up[0];
    int y = pixel_left_up[1];
    int width = abs(pixel_right_down[0] - pixel_left_up[0]);
    int height = abs(pixel_right_down[1] - pixel_left_up[1]);
    ROS_INFO_STREAM("width:"<<width << " height:"<<height);
    if (width == 0 || height == 0) {
      ROS_ERROR_STREAM("Error cropping image: width or height is 0");
      return;
    }
    // if (width+50>height || height +50>width) {
    //   ROS_ERROR_STREAM("is not a cube");
    //   return;
    // }
    try {
      cv::Mat croppedImg = cv::Mat(img, cv::Rect(x, y, width, height));

      // Convert cv::Mat to sensor_msgs::Image
      sensor_msgs::ImagePtr cropImage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", croppedImg).toImageMsg();
      cropImage->header.frame_id = "zed2_left_camera_optical_frame";
      cropImage->header.stamp = ros::Time::now();
      ROS_INFO_STREAM("---------------------Ready to publish---------------------");
      pc_pub_img.publish(cropImage);
      ROS_INFO_STREAM("------------------Cropped Image Published------------------");

      // Display diff
      // pc_pub_img.publish(cropImage);
      //cv::imshow( "Cropped Image",  croppedImg);
      cv::waitKey();
    } catch (cv::Exception &e) {
      ROS_ERROR_STREAM("Error cropping image: " << e.what());
    }
  }



  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0, 5.0);
    pass.filter(*cloud);
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
  
   /** \brief Given the pointcloud containing just the object,
      compute its center point, its height and its mesh and store in object_found.
      @param cloud - point cloud containing just then object. */
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
    ROS_WARN_STREAM("Max Z " << max_z_ << " Min Z " << min_z_);
    ROS_WARN_STREAM("Max X " << max_x_ << " Min X " << min_x_);
    ROS_WARN_STREAM("Max Y " << max_y_ << " Min Y " << min_y_);

    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);
    //ROS_WARN_STREAM("me iria mejor en el bote ");


    // Store the object centroid.
    object_found.center.header.frame_id = "map";
    object_found.center.pose.position.x = centroid.x;
    object_found.center.pose.position.y = centroid.y;
    object_found.center.pose.position.z = centroid.z;
    object_found.center.pose.orientation.x = 0.0;
    object_found.center.pose.orientation.y = 0.0;
    object_found.center.pose.orientation.z = 0.0;
    object_found.center.pose.orientation.w = 1.0;

    //Store the object centroid referenced to the camera frame.
    geometry_msgs::TransformStamped tf_to_cam;
    tf_to_cam = buffer_.lookupTransform(CAMERA_FRAME, "map", ros::Time(0), ros::Duration(1.0));
    tf2::doTransform(object_found.center, object_found.center_cam, tf_to_cam);

    if (isCluster) {
      object_found.isValid = true;
      // Add object only if is above table.
      // if (object_found.center.pose.position.z < table_params.min_z - 0.025) {
      //   ROS_INFO_STREAM("Object rejected due to table height: "
      //         << " x: " << object_found.center.pose.position.x
      //         << " y: " << object_found.center.pose.position.y
      //         << " z: " << object_found.center.pose.position.z
      //         << " Tz: " << table_params.min_z);
      //   object_found.isValid = false;
      //   return;
      // }

      ROS_INFO_STREAM("Object dimensions: "
        << " " << abs(object_found.max_x - object_found.min_x)
        << " " << abs(object_found.max_y - object_found.min_y)
        << " " << abs(object_found.max_z - object_found.min_z));

      // Add object only if it has restricted dimensions.
      if (abs(object_found.max_x - object_found.min_x) > 0.7 ||
        abs(object_found.max_y - object_found.min_y) > 0.7 ||
        abs(object_found.max_z - object_found.min_z) > 0.7) {
        ROS_INFO_STREAM("Object rejected due to dimensions.");
        object_found.isValid = false;
        return;
      }
      ROS_INFO_STREAM("-----------------Object accepted.-----------------");
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

    //removeBiggestPlane(cloud, inliers_plane, table_params);
    //extractNormals(cloud_normals, inliers_plane);

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

    std::vector<ObjectParams> objects;
    for(int i=0;i < clustersFound; i++)
    {
      ObjectParams tmp;
      extractObjectDetails(clusters[i], tmp, table_params);
      tmp.file_id = i;
      if (tmp.isValid) 
      {
        objects.push_back(tmp);
      }
    }

    if (objects.size() < 1) 
    {
      ROS_INFO_STREAM("objects array empty");
      return;
    }

        // Crop n Image to n objects and save them.
    for (int i = 0; i < objects.size(); i++) 
    {
      cropImage(zed_image_cv->image, objects[i]);
    }
    
          // for(int j = 0; j < cloud_rgb->width; j++)
          // {
          //   // Get the RGB values of the current point
          //     pcl::PointXYZRGB point = cloud_rgb->at(j, i);
          //     uint8_t r = point.r;
          //     uint8_t g = point.g;
          //     uint8_t b = point.b;
          //     //ROS_WARN("aqui mame %d", point.rgb);
          //   //ROS_WARN("aqui mame ");
          //   //ROS_WARN("aqui mame %d", k);
          //      // Store the RGB values in the image message data
          //     image_msg.data[k++] = r;
          //     image_msg.data[k++] = g;
          //     image_msg.data[k++] = b;
          //     // ROS_WARN("aqui mame r %d", r);
          //     // ROS_WARN("aqui mame g %d", g);
          //     // ROS_WARN("aqui mame b %d", b);
          // }
    } 
          //ROS_WARN("termine una");
        // cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
        // pc_pub_img.publish(cv_ptr->image);


    /** \brief Find all clusters in a pointcloud.*/
  void getClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &clusters) 
  {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    //Set parameters for the clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.035); // 3.5cm
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(20000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        cloud_cluster->points.push_back(cloud->points[*pit]); //*

      // Discard Noise
      if (cloud_cluster->points.size() < 50){
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


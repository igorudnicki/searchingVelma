#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "searcher/Segment.h"


double coords_global[3];

class CylinderSegment
{
  ros::NodeHandle nh_;

  ros::Subscriber cloud_subscriber_;


public:


  CylinderSegment()
    : cloud_subscriber_(nh_.subscribe("/head_kinect/depth/points", 1, &CylinderSegment::cloudCB, this))
  {
  }

  /** \brief Given the parameters of the cylinder add the cylinder to the planning scene. */
 void *addCylinder()
  {

    // Define a pose for the cylinder (specified relative to frame_id).
    geometry_msgs::Pose cylinder_pose;
    /* Computing and setting quaternion from axis angle representation. */
    Eigen::Vector3d cylinder_z_direction(cylinder_params.direction_vec[0], cylinder_params.direction_vec[1],
                                         cylinder_params.direction_vec[2]);
    Eigen::Vector3d origin_z_direction(0., 0., 1.);
    Eigen::Vector3d axis;
    axis = origin_z_direction.cross(cylinder_z_direction);
    axis.normalize();
    double angle = acos(cylinder_z_direction.dot(origin_z_direction));
    cylinder_pose.orientation.x = axis.x() * sin(angle / 2);
    cylinder_pose.orientation.y = axis.y() * sin(angle / 2);
    cylinder_pose.orientation.z = axis.z() * sin(angle / 2);
    cylinder_pose.orientation.w = cos(angle / 2);

    // Setting the position of cylinder.
    cylinder_pose.position.x = cylinder_params.center_pt[0];
    cylinder_pose.position.y = cylinder_params.center_pt[1];
    cylinder_pose.position.z = cylinder_params.center_pt[2];

    ros::Rate loop_rate(2);
    ros::Publisher poseStampedPub = nh_.advertise<geometry_msgs::PoseStamped>("marker", 2, true);

    double tol = 0.001;


    if (abs(cylinder_pose.position.y) > tol)
      {
        //cloud_subscriber_.shutdown();

        coords_global[0] = cylinder_pose.position.x;
        coords_global[1] = cylinder_pose.position.y;
        coords_global[2] = cylinder_pose.position.z;

        std::cout<<"X1: "<<coords_global[0]<<std::endl;
        std::cout<<"Y1: "<<coords_global[1]<<std::endl;
        std::cout<<"Z1: "<<coords_global[2]<<std::endl<<std::endl;
        /*

        while( ros::ok())
        {
          std::cout<<"Publishing marker"<<std::endl;
          geometry_msgs::PoseStamped poseStamped;
          poseStamped.header.frame_id="head_kinect_rgb_optical_frame";
          poseStamped.header.stamp = ros::Time::now();


          poseStamped.pose.position.x = x;
          poseStamped.pose.position.y = y;
          poseStamped.pose.position.z = z;

          tf2::Quaternion myQuaternion;

          double r =  0;
          double p = 0.68;
          double y = -1.6;

          myQuaternion.setRPY(r,p,y);

          myQuaternion=myQuaternion.normalize();


          poseStamped.pose.orientation.x = myQuaternion[0];
          poseStamped.pose.orientation.y = myQuaternion[1];
          poseStamped.pose.orientation.z = myQuaternion[2] ;
          poseStamped.pose.orientation.w = myQuaternion[3];

          poseStampedPub.publish(poseStamped);

          ros::spinOnce();

          loop_rate.sleep();
      }
      */
    }


  }

  /** \brief Given the pointcloud containing just the cylinder,
      compute its center point and its height and store in cylinder_params.
      @param cloud - point cloud containing just the cylinder. */
  void extractLocationHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    double max_angle_y = -std::numeric_limits<double>::infinity();
    double min_angle_y = std::numeric_limits<double>::infinity();

    double lowest_point[3] = { 0.0, 0.0, 0.0 };
    double highest_point[3] = { 0.0, 0.0, 0.0 };

    for (auto const point : cloud->points)
    {
      const double angle = atan2(point.z, point.y);
      /* Find the coordinates of the highest point */
      if (angle < min_angle_y)
      {
        min_angle_y = angle;
        lowest_point[0] = point.x;
        lowest_point[1] = point.y;
        lowest_point[2] = point.z;
      }
      /* Find the coordinates of the lowest point */
      else if (angle > max_angle_y)
      {
        max_angle_y = angle;
        highest_point[0] = point.x;
        highest_point[1] = point.y;
        highest_point[2] = point.z;
      }
    }
    /* Store the center point of cylinder */
    cylinder_params.center_pt[0] = (highest_point[0] + lowest_point[0]) / 2;
    cylinder_params.center_pt[1] = (highest_point[1] + lowest_point[1]) / 2;
    cylinder_params.center_pt[2] = (highest_point[2] + lowest_point[2]) / 2;
    /* Store the height of cylinder */
    cylinder_params.height =
        sqrt(pow((lowest_point[0] - highest_point[0]), 2) + pow((lowest_point[1] - highest_point[1]), 2) +
             pow((lowest_point[2] - highest_point[2]), 2));
    // END_SUB_TUTORIAL
  }

  /** \brief Given a pointcloud extract the ROI defined by the user.
      @param cloud - Pointcloud whose ROI needs to be extracted. */
  void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    // min and max values in z axis to keep
    pass.setFilterLimits(0.3, 1.1);
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
      @param inliers_plane - Indices whose normals need to be extracted. */
  void extractNormals(const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
                      const pcl::PointIndices::Ptr& inliers_plane)
  {
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(cloud_normals);
    extract_normals.setIndices(inliers_plane);
    extract_normals.filter(*cloud_normals);
  }

  /** \brief Given the pointcloud and indices of the plane, remove the planar region from the pointcloud.
      @param cloud - Pointcloud.
      @param inliers_plane - Indices representing the plane. */
  void removePlaneSurface(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointIndices::Ptr& inliers_plane)
  {
    // create a SAC segmenter without using normals
    pcl::SACSegmentation<pcl::PointXYZ> segmentor;
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_PLANE);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    /* run at max 1000 iterations before giving up */
    segmentor.setMaxIterations(1000);
    /* tolerance for variation from model */
    segmentor.setDistanceThreshold(0.01);
    segmentor.setInputCloud(cloud);
    /* Create the segmentation object for the planar model and set all the parameters */
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
    segmentor.segment(*inliers_plane, *coefficients_plane);
    /* Extract the planar inliers from the input cloud */
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers_plane);
    /* Remove the planar inliers, extract the rest */
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
  }


  void extractCylinder(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                       const pcl::ModelCoefficients::Ptr& coefficients_cylinder,
                       const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(1000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.008);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.04,0.055);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }

  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *cloud);
    // Use a passthrough filter to get the region of interest.
    // The filter removes points outside the specified range.
    passThroughFilter(cloud);
    // Compute point normals for later sample consensus step.
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    computeNormals(cloud, cloud_normals);
    // inliers_plane will hold the indices of the point cloud that correspond to a plane.
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
    // Detect and remove points on the (planar) surface on which the cylinder is resting.
    removePlaneSurface(cloud, inliers_plane);
    // Remove surface points from normals as well
    extractNormals(cloud_normals, inliers_plane);

    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    /* Extract the cylinder using SACSegmentation. */
    extractCylinder(cloud, coefficients_cylinder, cloud_normals);
    // END_SUB_TUTORIAL
    if (cloud->points.empty() || coefficients_cylinder->values.size() != 7)
    {
      //cloud_subscriber_.shutdown();
      ROS_ERROR_STREAM_NAMED("cylinder_segment", "Can't find the cylindrical component.");
      coords_global[0] = 0;
      coords_global[1] = 0;
      coords_global[2] = 0;
      //return;
    }



    cylinder_params.radius = coefficients_cylinder->values[6];
    /* Store direction vector of z-axis of cylinder. */
    cylinder_params.direction_vec[0] = coefficients_cylinder->values[3];
    cylinder_params.direction_vec[1] = coefficients_cylinder->values[4];
    cylinder_params.direction_vec[2] = coefficients_cylinder->values[5];

    extractLocationHeight(cloud);

    addCylinder();

  }
  

private:

  struct AddCylinderParams
  {
    /* Radius of the cylinder. */
    double radius;
    /* Direction vector along the z-axis of the cylinder. */
    double direction_vec[3];
    /* Center point of the cylinder. */
    double center_pt[3];
    /* Height of the cylinder. */
    double height;
  };
  AddCylinderParams cylinder_params;
};

bool segment_callback(searcher::Segment::Request  &req,
         searcher::Segment::Response &resp)
{

  resp.x = coords_global[0];
  resp.y = coords_global[1];
  resp.z = coords_global[2];

  ROS_INFO("sending back response x: [%f]", resp.x);
  return true;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "cylinder_segmentation");

  CylinderSegment segmentor;
  ros::NodeHandle n;
  std::cout<<"Working node"<<std::endl;

  ros::ServiceServer service = n.advertiseService("cylinder_segmentation", segment_callback);

  // Spin
  ros::spin();

}

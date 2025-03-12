// PCL Object Detection Node
// Detects objects resting on a Plane
// Currently, tested with objects on the floor


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/lexical_cast.hpp>

#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
// NOTE: you must install TF2 Sensor Messages: sudo apt-get install ros-kinetic-tf2-sensor-msgs



////////////////////////////////////////////////////////////////////////////////////////////////////////
class PclObjectDetection
{
public:
  PclObjectDetection(ros::NodeHandle n);

private:

  // FUNCTIONS
  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input_cloud_msg);
  void PublishMarkerBox(
        std::string frame_id, int id, float x, float y, float z, 
        float size_x, float size_y, float size_z, 
        float color_r, float color_g, float color_b);

  // VARIABLES
  ros::NodeHandle                 nh_;
  tf2_ros::Buffer                 tf2_;
  tf2_ros::TransformListener      tfListener_;
  std::string                     input_cloud_frame_;
  std::string                     base_cloud_frame_;


  // SUBSCRIBERS
  ros::Subscriber depth_cloud_sub_;

  // PUBLISHERS
  ros::Publisher pub_cluster0;
  ros::Publisher pub_cluster1;
  ros::Publisher pub_cluster2;
  ros::Publisher pub_cluster3;
  ros::Publisher pub_cluster4;
  ros::Publisher pub_cluster5;

  ros::Publisher pub0;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub5;
  
  ros::Publisher pub_remaining;
  ros::Publisher pub_objects;
  ros::Publisher marker_pub_;

};
////////////////////////////////////////////////////////////////////////////////////////////////////////

PclObjectDetection::PclObjectDetection(ros::NodeHandle n) : 
  nh_(n),
  //private_nh_("~"),
  tfListener_(tf2_),
  input_cloud_frame_(""),
  base_cloud_frame_("")
{

  ROS_INFO("PclObjectDetection: Initializing...");


  // PUBLISHERS
  // Create a ROS publisher for the output point cloud
  pub_cluster0 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster0", 1);
  pub_cluster1 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster1", 1);
  pub_cluster2 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster2", 1);
  pub_cluster3 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster3", 1);
  pub_cluster4 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster4", 1);
  pub_cluster5 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/cluster5", 1);

  pub0 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane0", 1);
  pub1 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane1", 1);
  pub2 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane2", 1);
  pub3 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane3", 1);
  pub4 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane4", 1);
  pub5 = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/plane5", 1);

  pub_remaining = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/remaining", 1);
  pub_objects = nh_.advertise<sensor_msgs::PointCloud2> ("pcl_object_detection/objects", 1);

  // Create a ROS publisher for the output model coefficients
  //pub = nh_.advertise<pcl_msgs::ModelCoefficients> ("pcl_test/segment_plane", 1);

  // Publish markers to show where robot thinks object is in RViz
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>
    ("pcl_test/marker", 1);


  // SUBSCRIBERS
  // Create a ROS subscriber for the input point cloud
  depth_cloud_sub_ = nh_.subscribe 
    ("/camera/points", 1, &PclObjectDetection::cloud_cb, this);


}

void PclObjectDetection::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud_msg)
{
  // ROS_INFO("PclObjectDetection: cloud_cb...");

  input_cloud_frame_ = input_cloud_msg->header.frame_id; // TF Frame of the point cloud
  // std::cout << "DEBUG Cloud Frame = [" << input_cloud_frame_ << "]" << std::endl;
  base_cloud_frame_ = "base_link";
  
 // CLOUD DATA STRUCTURES
  pcl::PCLPointCloud2::Ptr  
      cloud_blob (new pcl::PCLPointCloud2), 
      cloud_filtered_blob (new pcl::PCLPointCloud2);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr 
      downsampled_XYZ (new pcl::PointCloud<pcl::PointXYZ>),
      cloud_plane_p (new pcl::PointCloud<pcl::PointXYZ>), 
      cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Container for original & filtered data
  pcl::PCLPointCloud2* pcl2_input_cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr pcl2_input_cloud_p(pcl2_input_cloud);
  pcl::PCLPointCloud2 cloud_filtered2;

  // Convert from ROS to PCL2 cloud
  pcl_conversions::toPCL(*input_cloud_msg, *pcl2_input_cloud);

  // Perform the actual filtering (reduce points by clumping into voxels
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (pcl2_input_cloud_p);
  sor.setLeafSize (0.01, 0.01, 0.01); // 1 cm voxels
  sor.setFilterFieldName ("z");
  sor.setFilterLimits (0.1, 1.5);
  sor.setDownsampleAllData (false);
  sor.filter (cloud_filtered2);

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  //pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  //pcl::fromROSMsg (cloud_filtered2, cloud_filtered);
  
  // Convert from PCL2 to PCL XYZ cloud?
  pcl::fromPCLPointCloud2(cloud_filtered2, *downsampled_XYZ);

  //std::cout << "PointCloud after filtering: " << downsampled_XYZ->width * downsampled_XYZ->height << " data points." << std::endl;

  // Segment the Plane (Floor)  ... (or Tabletop??? TODO)
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  int i = 0, nr_points = (int) downsampled_XYZ->points.size ();
  // While 30% of the original cloud is still there
  while (downsampled_XYZ->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (downsampled_XYZ);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    // Plane equation:  ax + by +cz + d = 0
    /*
    std::cerr << "Plane " << i << " coefficients: " 
      << coefficients->values[0] << " x " 
      << coefficients->values[1] << " y "
      << coefficients->values[2] << " z " 
      << coefficients->values[3] << " d "<< std::endl;
    */    
    
    /*
      // Publish the model coefficients for each plane
      pcl_msgs::ModelCoefficients ros_coefficients;
      pcl_conversions::fromPCL(*coefficients, ros_coefficients);
      pub.publish (ros_coefficients);
    */  
 
    // Extract the inliers
    extract.setInputCloud (downsampled_XYZ);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_plane_p);
    //std::cout << "PointCloud representing the planar component [" << i << "] : " << cloud_plane_p->width * cloud_plane_p->height << " data points." << std::endl;


  // TODO?  Try using Hull to better segment the plane?
//#define DO_HULL
#ifdef DO_HULL

    if( (cloud_plane_p->width > 0) && (cloud_plane_p->height > 0) )
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new 
          pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new 
          pcl::PointCloud<pcl::PointXYZ>);

      // Retrieve the convex hull.
	    pcl::ConvexHull<pcl::PointXYZ> hull;
	    hull.setInputCloud(cloud_plane_p);
	    // Make sure that the resulting hull is bidimensional.
	    hull.setDimension(2);
	    hull.reconstruct(*convexHull);

	    // Redundant check.
	    if (hull.getDimension() == 2)
	    {
		    // Prism object.
		    pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
		    prism.setInputCloud(downsampled_XYZ);
		    prism.setInputPlanarHull(convexHull);
		    // First parameter: minimum Z value. Set to 0, segments objects lying on the plane (can be negative).
		    // Second parameter: maximum Z value, set to 10cm. Tune it according to the height of the objects you expect.
		    prism.setHeightLimits(-0.1f, 0.03f);
		    pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

		    prism.segment(*objectIndices);

		    // Get and show all points retrieved by the hull.
		    extract.setIndices(objectIndices);
		    extract.filter(*objects);
		    //pcl::visualization::CloudViewer viewerObjects("Objects on table");
		    //viewerObjects.showCloud(objects);
		
        sensor_msgs::PointCloud2 obj_output;
        pcl::PCLPointCloud2 obj_tmp_cloud;
        pcl::toPCLPointCloud2(*objects, obj_tmp_cloud);
        pcl_conversions::fromPCL(obj_tmp_cloud, obj_output);
		    pub_objects.publish (obj_output); // Publish the data
		
	    }
	    else std::cout << "The chosen hull is not planar." << std::endl;
    }
#endif // DO_HULL


    if(i < 5)
    {
      // Publish Plane point clouds for debug
      sensor_msgs::PointCloud2 output;
      pcl::PCLPointCloud2 tmp_cloud;
      pcl::toPCLPointCloud2(*cloud_plane_p, tmp_cloud);
      pcl_conversions::fromPCL(tmp_cloud, output);

      //std::cout << "Publishing [" << i << "]" << std::endl;
      
      if(i == 0)
      {
        pub0.publish (output); // Publish the data
      }
      if(i == 1)
      {
        pub1.publish (output); // Publish the data
      }
      else if(i == 2)
      {
        pub2.publish (output); // Publish the data
      }
      else if(i == 3)
      {
        pub3.publish (output); // Publish the data
      }
      else if(i == 4)
      {
        pub4.publish (output); // Publish the data
      }
    } 
    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    downsampled_XYZ.swap (cloud_f);
    i++;
  }
 
  // See what's left:
  // std::cout << "PointCloud after segment removal: " << 
  //  downsampled_XYZ->width * downsampled_XYZ->height << " data points." << std::endl;

  // Convert and publish the cloud of remaining points (outside the plane)
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 tmp_cloud;
  pcl::toPCLPointCloud2(*downsampled_XYZ, tmp_cloud);
  pcl_conversions::fromPCL(tmp_cloud, output);
  pub_remaining.publish (output); 
 

  // Look for object Clusters

  if( (downsampled_XYZ->width > 0) && (downsampled_XYZ->height > 0) )
  {
 
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (downsampled_XYZ);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.050); // 50 millimeters
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (downsampled_XYZ);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (downsampled_XYZ->points[*pit]);

        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "Cluster: " << j << " : " 
        //  << cloud_cluster->points.size() << " data points." << std::endl;
        
        //Convert the pointcloud to be used in ROS
        sensor_msgs::PointCloud2::Ptr output (new sensor_msgs::PointCloud2);
        pcl::toROSMsg (*cloud_cluster, *output);
        output->header.frame_id = input_cloud_frame_;
 
        // Rotate the clusters found
        sensor_msgs::PointCloud2Ptr cloud_rotated_msg;
        pcl::PCLPointCloud2 pcl2_rotated_cluster; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_rotated_cluster_XYZ (new pcl::PointCloud<pcl::PointXYZ>);
        double  tf_tolerance_ = 0.05; 
        pcl::PointXYZ minPt, maxPt, bb_size, obj_center;
        
        try
        {
          cloud_rotated_msg.reset(new sensor_msgs::PointCloud2);
          tf2_.transform(*output, *cloud_rotated_msg, base_cloud_frame_, ros::Duration(tf_tolerance_));

          // Convert from ROS to PCL2 cloud
          pcl_conversions::toPCL(*cloud_rotated_msg, pcl2_rotated_cluster);
          // Convert from PCL2 to PCL XYZ cloud? (surely there is a better way!?)
          pcl::fromPCLPointCloud2(pcl2_rotated_cluster, *pcl_rotated_cluster_XYZ);

          // Find bounding box of rotated cluster
          pcl::getMinMax3D (*pcl_rotated_cluster_XYZ, minPt, maxPt);
          /*
          std::cout << "CLUSTER [" << j << "] : "
            << "Max x: " << maxPt.x 
            << ", y: " << maxPt.y 
            << ", z: " << maxPt.z 
            << "    Min x: " << minPt.x 
            << ", y: " << minPt.y 
            << ", z: " << minPt.z << std::endl;
          */
            
          bb_size.x = maxPt.x - minPt.x;
          bb_size.y = maxPt.y - minPt.y;
          bb_size.z = maxPt.z - minPt.z;

          // Filter object size
          if( (maxPt.z   < 0.020) ||    // Min heigth of object is below noise floor
              (bb_size.z < 0.030) ||    // Min heigth of object
              (maxPt.z   > 0.200) ||    // Max heigth of object
              (minPt.z   > 0.040) )     // bottom of object
          {
            continue; // Skip objects that don't meet size criteria (could be a wall, etc.)
          }

          obj_center.x = minPt.x + (bb_size.x / 2);
          obj_center.y = minPt.y + (bb_size.y / 2);
          obj_center.z = minPt.z + (bb_size.z / 2);

          std::cout << "OBJECT [" << j << "] : "
            << "  x: " << obj_center.x
            << ", y: " << obj_center.y 
            << ", z: " << obj_center.z               
            << ", l: " << bb_size.x 
            << ", w: " << bb_size.y 
            << ", h: " << bb_size.z 
            << ", top: " << maxPt.z 
            << ", bottom: " << minPt.z                             
            << std::endl;
            
          if(j < 6)
          {
            if(j == 0)
            {
              pub_cluster0.publish (cloud_rotated_msg);   // Publish the data cluster cloud

              PclObjectDetection::PublishMarkerBox(       // Publish the bounding box as a marker
                base_cloud_frame_,                        // Transform Frame from camera to robot base
                j,                                        // Marker ID
                obj_center.x, obj_center.y, obj_center.z, // Object Center 
                bb_size.x, bb_size.y, bb_size.z,          // Object Size
                1.0, 0.0, 0.0 ); // Red                   // r,g,b - different for each marker
            }
            else if(j == 1)
            {
              pub_cluster1.publish (cloud_rotated_msg); 

              PclObjectDetection::PublishMarkerBox(     
                base_cloud_frame_,    
                j, 
                obj_center.x, obj_center.y, obj_center.z,   
                bb_size.x, bb_size.y, bb_size.z,            
                0.0, 1.0, 0.0 ); // Green
            }
            else if(j == 2)
            {
              pub_cluster2.publish (cloud_rotated_msg); 

              PclObjectDetection::PublishMarkerBox(     
                base_cloud_frame_,    
                j, 
                obj_center.x, obj_center.y, obj_center.z,   
                bb_size.x, bb_size.y, bb_size.z,            
                1.0, 0.0, 1.0 ); // Light Purple
            }
            else if(j == 3)
            {
              pub_cluster3.publish (cloud_rotated_msg); 

              PclObjectDetection::PublishMarkerBox(     
                base_cloud_frame_,    
                j, 
                obj_center.x, obj_center.y, obj_center.z,   
                bb_size.x, bb_size.y, bb_size.z,            
                1.0, 1.0, 0.0 ); // Yellow
            }
            else if(j == 4)
            {
              pub_cluster4.publish (cloud_rotated_msg); 

              PclObjectDetection::PublishMarkerBox(     
                base_cloud_frame_,    
                j, 
                obj_center.x, obj_center.y, obj_center.z,   
                bb_size.x, bb_size.y, bb_size.z,            
                0.0, 1.0, 1.0 ); // Aqua
            }
            else if(j == 5)
            {
              pub_cluster5.publish (cloud_rotated_msg); 

              PclObjectDetection::PublishMarkerBox(     
                base_cloud_frame_,    
                j, 
                obj_center.x, obj_center.y, obj_center.z,   
                bb_size.x, bb_size.y, bb_size.z,            
                0.5, 0.0, 0.5 ); // Dark Purple
            }
          }       
              
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN_STREAM("Transform failure: " << ex.what());
          break;
        }
        
        ++j;
    }
  }
  
} // cloud_cb


void PclObjectDetection::PublishMarkerBox(
      std::string frame_id, int id, float x, float y, float z, 
      float size_x, float size_y, float size_z, 
      float color_r, float color_g, float color_b)
{
  // Display marker for RVIZ to show where robot thinks person is
  // For Markers info, see http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

  // ROS_INFO("DBG: PublishMarkerBox called");
  //if( id != 1)
  // printf ("DBG PublishMarkerBox called for %f, %f, %f\n", x,y,z);

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id; // input_cloud_frame_; //"camera_depth_frame"; //"base_link";
  marker.header.stamp = ros::Time::now();
  marker.lifetime = ros::Duration(3.0); // seconds
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "pcl_object_detection";
  marker.id = id; // This must be id unique for each marker

  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and DELETEALL
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = color_r;
  marker.color.g = color_g; 
  marker.color.b = color_b;
  marker.color.a = 0.75;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = size_x; // size of marker in meters
  marker.scale.y = size_y;
  marker.scale.z = size_z;  

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;

  // ROS_INFO("DBG: Publishing Marker");
  marker_pub_.publish(marker);

}




int main (int argc, char** argv)
{

  ROS_INFO("PclObjectDetection: Initializing ROS... ");
  ros::init(argc, argv, "pcl_object_detection");

  ros::NodeHandle n;
  //ros::NodeHandle n("~");  
  PclObjectDetection pcl_object_detection_node(n);
  ros::spin();
  return 0;

}

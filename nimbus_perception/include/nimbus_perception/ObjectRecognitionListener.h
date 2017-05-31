#ifndef OBJECT_RECOGNITION_LISTENER_H_
#define OBJECT_RECOGNITION_LISTENER_H_

// ROS
#include <nimbus_perception/Classify.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>

// Boost
#include <boost/thread/mutex.hpp>

class ObjectRecognitionListener
{
public:
  static const float DOWNSAMPLE_LEAF_SIZE = 0.01;
  static const double SAME_OBJECT_DIST_THRESHOLD = 0.2;

  ObjectRecognitionListener();

  bool okay() const;

private:
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objectList);

  void combineModels(const rail_manipulation_msgs::SegmentedObject &model1,
      const rail_manipulation_msgs::SegmentedObject &model2, rail_manipulation_msgs::SegmentedObject &combined) const;


  boost::mutex objectsMutex;

  ros::NodeHandle n, pnh;
  ros::Subscriber segmentedObjectsSubscriber;
  ros::Publisher recognizedObjectsPublisher;
  ros::ServiceClient classifyClient;
  rail_manipulation_msgs::SegmentedObjectList objects;
};

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB);

#endif

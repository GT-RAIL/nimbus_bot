#ifndef OBJECT_LABELER_H_
#define OBJECT_LABELER_H_

//ROS
#include <nimbus_perception/ClassifyInstance.h>
#include <nimbus_perception/TrackedObjectList.h>
#include <pcl_ros/point_cloud.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <shape_msgs/SolidPrimitive.h>

//PCL
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

//C++
#include <boost/thread/mutex.hpp>

class ObjectLabeler
{

public:

  /**
   * \brief Constructor
   */
  ObjectLabeler();

private:
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData);

  nimbus_perception::TrackedObject calculateMeasurement(rail_manipulation_msgs::SegmentedObject object);

  std::string disambiguateSpecialCase(nimbus_perception::TrackedObject object);

  ros::NodeHandle n, pnh;

  ros::Subscriber newDataSubscriber;

  ros::Publisher objectMeasurementPublisher;

  ros::ServiceClient recognizeClient;
};

#endif

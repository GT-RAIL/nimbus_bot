#ifndef NIMBUS_OBJECT_VIEWER_H_
#define NIMBUS_OBJECT_VIEWER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/thread/recursive_mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/GripperAction.h>
#include <rail_manipulation_msgs/LiftAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_pick_and_place_msgs/RemoveObject.h>
#include <rail_segmentation/RemoveObject.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_srvs/Empty.h>
#include <wpi_jaco_msgs/CartesianCommand.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/QuaternionToEuler.h>

class NimbusObjectViewer
{

public:

  /**
   * \brief Constructor
   */
  NimbusObjectViewer();

  /**
   * \brief callback for segmented objects to be displayed
   * @param objectList list of segmented objects for displaying
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr& objectList);

  /**
   * \brief clear all segmented objects from the interactive marker server
   */
  void clearSegmentedObjects();

private:
  ros::NodeHandle n;

  //messages
  ros::Subscriber recognizedObjectsSubscriber;

  boost::recursive_mutex api_mutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server
  std::vector<visualization_msgs::InteractiveMarker> segmentedObjects;  //!< list of segmented objects as interactive markers
  rail_manipulation_msgs::SegmentedObjectList segmentedObjectList;  //!< list of segmented objects in the rail_manipulation_msgs form
};

#endif

#ifndef NIMBUS_INTERACTIVE_MANIPULATION_H_
#define NIMBUS_INTERACTIVE_MANIPULATION_H_

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

class NimbusInteractiveManipulation
{

public:

  /**
   * \brief Constructor
   */
  NimbusInteractiveManipulation();

  void updateJoints(const sensor_msgs::JointState::ConstPtr& msg);

  void makeHandMarker();

  /**
  * \brief Process feedback for the interactive marker on the JACO's end effector
  * @param feedback interactive marker feedback
  */
  void processHandMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
  * \brief Update the interactive marker on the JACO's end effector to move based on the the current joint state of the arm
  */
  void updateMarkerPosition();

  /**
   * /brief Process feedback for objects that can be recognized.
   * @param feedback interactive marker feedback
   */
  //void processRecognizeMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * /brief Process feedback for objects that can be picked up.
   * @param feedback interactive marker feedback
   */
  void processPickupMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
  * \brief Process feedback for objects that are selected for removal.
  * @param feedback interactive marker feedback
  */
  void processRemoveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * \brief callback for segmented objects to be displayed
   * @param objectList list of segmented objects for displaying
   */
  void segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr& objectList);

  /**
   * \brief clear all segmented objects from the interactive marker server
   */
  void clearSegmentedObjects();

  /**
  * \brief Send a 0 velocity command to the robot's arm
  */
  void sendStopCommand();

private:

  /**
  * \brief Remove a manipulation object marker
  * @param index object index
  * @return true if object was successfully removed
  */
  bool removeObjectMarker(int index);

  ros::NodeHandle n;

  //messages
  ros::Publisher cartesianCmd;
  ros::Subscriber jointStateSubscriber;
  ros::Subscriber recognizedObjectsSubscriber;

  //services
  ros::ServiceClient removeObjectClient;
  ros::ServiceClient detachObjectsClient;
  ros::ServiceClient jacoFkClient;
  ros::ServiceClient qeClient;
  ros::ServiceClient eraseTrajectoriesClient;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> acPickup;
  actionlib::SimpleActionClient<rail_manipulation_msgs::GripperAction> acGripper;
  actionlib::SimpleActionClient<rail_manipulation_msgs::LiftAction> acLift;
  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> acArm;

  boost::recursive_mutex api_mutex;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server
  interactive_markers::MenuHandler menuHandler; //!< interactive marker menu handler
  interactive_markers::MenuHandler objectMenuHandler; //!< object interactive markers menu handler
  std::vector<interactive_markers::MenuHandler> recognizedMenuHandlers; //!< list of customized menu handlers for recognized objects
  std::vector<visualization_msgs::InteractiveMarker> segmentedObjects;  //!< list of segmented objects as interactive markers
  rail_manipulation_msgs::SegmentedObjectList segmentedObjectList;  //!< list of segmented objects in the rail_manipulation_msgs form

  bool usingRecognition;
  bool usingPickup;
  bool lockPose;
  std::vector<float> joints;
};

#endif

#ifndef NIMBUS_6DOF_VIS_H_
#define NIMBUS_6DOF_VIS_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nimbus_interactive_manipulation/SpecifiedGraspAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <wpi_jaco_msgs/JacoFK.h>

class Nimbus6dofVis
{

public:

  /**
   * \brief Constructor
   */
  Nimbus6dofVis();

private:

  void makeGripperMarker();

  void markerPositionCallback(const geometry_msgs::PoseStamped& pose);

  /**
  * \brief Update the interactive marker on the JACO's end effector to move based on the the current joint state of the arm
  */
  void updateMarkerPosition();

  ros::NodeHandle n, pnh;

  //messages
  ros::Subscriber markerPoseSubscriber;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server
};

#endif

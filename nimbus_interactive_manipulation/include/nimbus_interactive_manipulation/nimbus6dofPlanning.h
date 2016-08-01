#ifndef NIMBUS_6DOF_PLANNING_H_
#define NIMBUS_6DOF_PLANNING_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <interactive_markers/interactive_marker_server.h>
#include <nimbus_interactive_manipulation/SpecifiedGraspAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <wpi_jaco_msgs/JacoFK.h>

class Nimbus6dofPlanning
{

public:

  /**
   * \brief Constructor
   */
  Nimbus6dofPlanning();

  void publishMarkerPosition();

private:

  void makeGripperMarker();

  void updateJoints(const sensor_msgs::JointState::ConstPtr& msg);

  bool resetMarkerPositionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void executeGraspCallback(const nimbus_interactive_manipulation::SpecifiedGraspGoalConstPtr &goal);

  ros::NodeHandle n, pnh;

  //messages
  ros::Publisher markerPosePublisher;

  //services
  ros::ServiceServer resetMarkerPositionServer;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupUnrecognizedClient;
  actionlib::SimpleActionServer<nimbus_interactive_manipulation::SpecifiedGraspAction> specifiedGraspServer;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server

  tf::TransformListener tfListener;
};

#endif

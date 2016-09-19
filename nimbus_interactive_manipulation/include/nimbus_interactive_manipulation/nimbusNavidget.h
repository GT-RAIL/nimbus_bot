#ifndef NIMBUS_NAVIDGET_H_
#define NIMBUS_NAVIDGET_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/recursive_mutex.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <nimbus_interactive_manipulation/SpecifiedGraspAction.h>
#include <nimbus_interactive_manipulation/CreateNavidgetSphere.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

class NimbusNavidget
{

public:

  /**
   * \brief Constructor
   */
  NimbusNavidget();

private:

  void makeGripperMarker(geometry_msgs::Point point, geometry_msgs::QuaternionStamped orientation);

  bool clearGripperMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool clearFullMarkerCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool createSphereMarkerCallback(nimbus_interactive_manipulation::CreateNavidgetSphere::Request &req, nimbus_interactive_manipulation::CreateNavidgetSphere::Response &res);

  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void executeGraspCallback(const nimbus_interactive_manipulation::SpecifiedGraspGoalConstPtr &goal);

  visualization_msgs::Marker createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, std::string meshLocation);

  ros::NodeHandle n, pnh;

  //services
  ros::ServiceServer clearGripperMarkerServer;
  ros::ServiceServer clearFullMarkerServer;
  ros::ServiceServer createNavidgetServer;

  //actionlib
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupUnrecognizedClient;
  actionlib::SimpleActionServer<nimbus_interactive_manipulation::SpecifiedGraspAction> specifiedGraspServer;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer; //!< interactive marker server

  boost::recursive_mutex graspMutex;

  tf::TransformListener tfListener;
};

#endif

#include <nimbus_interactive_manipulation/nimbus6dofPlanning.h>

using namespace std;

Nimbus6dofPlanning::Nimbus6dofPlanning() :
        pnh("~"), pickupUnrecognizedClient("nimbus_moveit/common_actions/pickup_unrecognized"),
        specifiedGraspServer(pnh, "execute_grasp", boost::bind(&Nimbus6dofPlanning::executeGraspCallback, this, _1), false)
{
  //messages
  markerPosePublisher = pnh.advertise<geometry_msgs::PoseStamped>("gripper_marker_pose", 1);

  //services
  resetMarkerPositionServer = pnh.advertiseService("reset_marker_position", &Nimbus6dofPlanning::resetMarkerPositionCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_6dof_planning", "nimbus_6dof_planning", false));

  ros::Duration(0.1).sleep();

  makeGripperMarker();

  imServer->applyChanges();

  specifiedGraspServer.start();
}

void Nimbus6dofPlanning::makeGripperMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "table_base_link";

  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform("nimbus_ee_link", "table_base_link", ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform("table_base_link", "nimbus_ee_link", ros::Time(0), currentEefTransform);
  iMarker.pose.position.x = currentEefTransform.getOrigin().x();
  iMarker.pose.position.y = currentEefTransform.getOrigin().y();
  iMarker.pose.position.z = currentEefTransform.getOrigin().z();
  iMarker.pose.orientation.x = currentEefTransform.getRotation().x();
  iMarker.pose.orientation.y = currentEefTransform.getRotation().y();
  iMarker.pose.orientation.z = currentEefTransform.getRotation().z();
  iMarker.pose.orientation.w = currentEefTransform.getRotation().w();

  iMarker.scale = 0.2;

  iMarker.name = "nimbus_gripper";
  iMarker.description = "Set Nimbus' gripper pose";

  //make a shape primitive representation of a gripper
  visualization_msgs::Marker gripperMarker;
  gripperMarker.pose.orientation.w = 1.0;
  gripperMarker.type = visualization_msgs::Marker::CUBE;
  gripperMarker.scale.x = 0.076;
  gripperMarker.scale.y = 0.02;
  gripperMarker.scale.z = 0.11;
  gripperMarker.color.r = 0.75;
  gripperMarker.color.g = 0.0;
  gripperMarker.color.b = 0.75;
  gripperMarker.color.a = 0.5;
//  visualization_msgs::Marker gripperMarker;
//  gripperMarker.pose.position.x = -0.125;
//  gripperMarker.pose.orientation.x = 0.7071;
//  gripperMarker.pose.orientation.w = 0.7071;
//  gripperMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
//  gripperMarker.mesh_resource = "package://agile_test_nodes/meshes/robotiq_85_base_link.dae";
//  gripperMarker.scale.x = 1.0;
//  gripperMarker.scale.y = 1.0;
//  gripperMarker.scale.z = 1.0;
//  gripperMarker.color.r = 1.0;
//  gripperMarker.color.g = 1.0;
//  gripperMarker.color.b = 0.0;
//  gripperMarker.color.a = 0.5;
  visualization_msgs::Marker gripperFingerL;
  gripperFingerL.pose.position.x = 0.065;
  gripperFingerL.pose.position.z = -0.05;
  gripperFingerL.pose.orientation.w = 1.0;
  gripperFingerL.type = visualization_msgs::Marker::CUBE;
  gripperFingerL.scale.x = 0.052;
  gripperFingerL.scale.y = 0.02;
  gripperFingerL.scale.z = 0.01;
  gripperFingerL.color.r = 0.75;
  gripperFingerL.color.g = 0.0;
  gripperFingerL.color.b = 0.75;
  gripperFingerL.color.a = 0.5;
  visualization_msgs::Marker gripperFingerR;
  gripperFingerR.pose.position.x = 0.065;
  gripperFingerR.pose.position.z = 0.05;
  gripperFingerR.pose.orientation.w = 1.0;
  gripperFingerR.type = visualization_msgs::Marker::CUBE;
  gripperFingerR.scale.x = 0.052;
  gripperFingerR.scale.y = 0.02;
  gripperFingerR.scale.z = 0.01;
  gripperFingerR.color.r = 0.75;
  gripperFingerR.color.g = 0.0;
  gripperFingerR.color.b = 0.75;
  gripperFingerR.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back(gripperMarker);
  gripperControl.markers.push_back(gripperFingerL);
  gripperControl.markers.push_back(gripperFingerR);
  gripperControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripperControl.name = "gripper_control";
  gripperControl.always_visible = true;

  iMarker.controls.push_back(gripperControl);

  //add 6-DOF controls
  visualization_msgs::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  iMarker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  iMarker.controls.push_back(control);

  imServer->insert(iMarker);
}

void Nimbus6dofPlanning::executeGraspCallback(const nimbus_interactive_manipulation::SpecifiedGraspGoalConstPtr &goal)
{
  nimbus_interactive_manipulation::SpecifiedGraspFeedback feedback;
  nimbus_interactive_manipulation::SpecifiedGraspResult result;

  feedback.message = "Moving arm to your set position...";
  specifiedGraspServer.publishFeedback(feedback);

  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("nimbus_gripper", poseMarker);

  rail_manipulation_msgs::PickupGoal pickupGoal;
  pickupGoal.pose.header = poseMarker.header;
  pickupGoal.pose.pose = poseMarker.pose;
  pickupGoal.lift = false;
  pickupGoal.verify = false;
  pickupUnrecognizedClient.sendGoal(pickupGoal);
  pickupUnrecognizedClient.waitForResult(ros::Duration(30.0));
  rail_manipulation_msgs::PickupResultConstPtr pickupResult = pickupUnrecognizedClient.getResult();
  result.success = pickupResult->success;
  result.executionSuccess = pickupResult->executionSuccess;
  if (!pickupResult->executionSuccess)
  {
    ROS_INFO("Grasp failed!");
    feedback.message = "The robot failed to plan to your selected grasp.";
  }
  else
  {
    ROS_INFO("Grasp succeeded.");
    feedback.message = "Success!";
  }
  specifiedGraspServer.publishFeedback(feedback);
  specifiedGraspServer.setSucceeded(result);
}

bool Nimbus6dofPlanning::resetMarkerPositionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  tf::StampedTransform currentEefTransform;
  tfListener.waitForTransform("nimbus_ee_link", "table_base_link", ros::Time::now(), ros::Duration(1.0));
  tfListener.lookupTransform("table_base_link", "nimbus_ee_link", ros::Time(0), currentEefTransform);
  geometry_msgs::Pose pose;
  pose.position.x = currentEefTransform.getOrigin().x();
  pose.position.y = currentEefTransform.getOrigin().y();
  pose.position.z = currentEefTransform.getOrigin().z();
  pose.orientation.x = currentEefTransform.getRotation().x();
  pose.orientation.y = currentEefTransform.getRotation().y();
  pose.orientation.z = currentEefTransform.getRotation().z();
  pose.orientation.w = currentEefTransform.getRotation().w();
  imServer->setPose("nimbus_gripper", pose);
  imServer->applyChanges();

  return true;
}

void Nimbus6dofPlanning::publishMarkerPosition()
{
  visualization_msgs::InteractiveMarker poseMarker;
  imServer->get("nimbus_gripper", poseMarker);

  geometry_msgs::PoseStamped pose;
  pose.header = poseMarker.header;
  pose.pose = poseMarker.pose;

  markerPosePublisher.publish(pose);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_6dof_planning");

  Nimbus6dofPlanning n6p;

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    n6p.publishMarkerPosition();
    ros::spinOnce();
    loopRate.sleep();
  }
}

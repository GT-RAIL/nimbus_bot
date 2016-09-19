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

  //gripper mesh marker
  visualization_msgs::Marker gripperBase = createGripperMeshMarker(
      -0.055, 0, 0,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_base_link.dae");
  visualization_msgs::Marker gripperLeftKnuckle = createGripperMeshMarker(
      -0.001, 0, -0.031,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperRightKnuckle = createGripperMeshMarker(
      -0.001, 0, 0.031,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFinger = createGripperMeshMarker(
      -0.005, 0, -0.062,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperRightFinger = createGripperMeshMarker(
      -0.005, 0, 0.062,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperLeftInnerKnuckle = createGripperMeshMarker(
      0.006, 0, -0.013,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperRightInnerKnuckle = createGripperMeshMarker(
      0.006, 0, 0.013,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFingerTip = createGripperMeshMarker(
      0.049, 0, -0.05,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae");
  visualization_msgs::Marker gripperRightFingerTip = createGripperMeshMarker(
      0.049, 0, 0.05,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/visual/robotiq_85_finger_tip_link.dae");

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back(gripperBase);
  gripperControl.markers.push_back(gripperLeftKnuckle);
  gripperControl.markers.push_back(gripperRightKnuckle);
  gripperControl.markers.push_back(gripperLeftFinger);
  gripperControl.markers.push_back(gripperRightFinger);
  gripperControl.markers.push_back(gripperLeftInnerKnuckle);
  gripperControl.markers.push_back(gripperRightInnerKnuckle);
  gripperControl.markers.push_back(gripperLeftFingerTip);
  gripperControl.markers.push_back(gripperRightFingerTip);
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

visualization_msgs::Marker Nimbus6dofPlanning::createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, string meshLocation)
{
  visualization_msgs::Marker marker;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = rx;
  marker.pose.orientation.y = ry;
  marker.pose.orientation.z = rz;
  marker.pose.orientation.w = rw;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = meshLocation;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.85;
  marker.color.g = 0.0;
  marker.color.b = 0.85;
  marker.color.a = 0.5;

  return marker;
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
    feedback.message = "Failed to plan to your grasp. Try a different start or end pose, and watch out for collisions.";
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

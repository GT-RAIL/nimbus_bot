#include <nimbus_interactive_manipulation/nimbus6dofVis.h>

using namespace std;

Nimbus6dofVis::Nimbus6dofVis() : pnh("~")
{
  //messages
  markerPoseSubscriber = n.subscribe("nimbus_6dof_planning/gripper_marker_pose", 1, &Nimbus6dofVis::markerPositionCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_6dof_vis", "nimbus_6dof_vis", false));

  ros::Duration(0.1).sleep();

  makeGripperMarker();

  imServer->applyChanges();
}

void Nimbus6dofVis::markerPositionCallback(const geometry_msgs::PoseStamped& pose)
{
  imServer->setPose("nimbus_gripper", pose.pose);
  imServer->applyChanges();
}

void Nimbus6dofVis::makeGripperMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "jaco_base_link";

  iMarker.pose.position.x = 0.0;
  iMarker.pose.position.y = 0.0;
  iMarker.pose.position.z = 0.0;
  iMarker.pose.orientation.x = 0.0;
  iMarker.pose.orientation.y = 0.0;
  iMarker.pose.orientation.z = 0.0;
  iMarker.pose.orientation.w = 1.0;
  iMarker.scale = 1.0;

  iMarker.name = "nimbus_gripper";
  iMarker.description = "View Nimbus' planned gripper pose";

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

  imServer->insert(iMarker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_6dof_vis");

  Nimbus6dofVis n6v;

  ros::spin();
}

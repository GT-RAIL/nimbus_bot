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
}

void Nimbus6dofVis::makeGripperMarker()
{
  visualization_msgs::InteractiveMarker iMarker;
  iMarker.header.frame_id = "table_base_link";

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

  visualization_msgs::Marker gripperMarker;
  gripperMarker.pose.position.x = -0.125;
  gripperMarker.pose.orientation.x = 0.7071;
  gripperMarker.pose.orientation.w = 0.7071;
  gripperMarker.type = visualization_msgs::Marker::MESH_RESOURCE;
  gripperMarker.mesh_resource = "package://agile_test_nodes/meshes/robotiq_85_base_link.dae";
  gripperMarker.scale.x = 1.0;
  gripperMarker.scale.y = 1.0;
  gripperMarker.scale.z = 1.0;
  gripperMarker.color.r = 1.0;
  gripperMarker.color.g = 1.0;
  gripperMarker.color.b = 0.0;
  gripperMarker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl gripperControl;
  gripperControl.markers.push_back(gripperMarker);
  gripperControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  gripperControl.name = "gripper_control";

  iMarker.controls.push_back(gripperControl);

  imServer->insert(iMarker);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_6dof_vis");

  Nimbus6dofVis n6v;

  ros::spin();
}

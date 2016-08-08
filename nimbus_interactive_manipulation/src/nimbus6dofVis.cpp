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

  //gripper mesh marker
  visualization_msgs::Marker gripperBase = createGripperMeshMarker(
      -0.055, 0, 0,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_base_link.dae");
  visualization_msgs::Marker gripperLeftKnuckle = createGripperMeshMarker(
      -0.001, 0, -0.031,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperRightKnuckle = createGripperMeshMarker(
      -0.001, 0, 0.031,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFinger = createGripperMeshMarker(
      -0.005, 0, -0.062,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperRightFinger = createGripperMeshMarker(
      -0.005, 0, 0.062,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_finger_link.dae");
  visualization_msgs::Marker gripperLeftInnerKnuckle = createGripperMeshMarker(
      0.006, 0, -0.013,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperRightInnerKnuckle = createGripperMeshMarker(
      0.006, 0, 0.013,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_inner_knuckle_link.dae");
  visualization_msgs::Marker gripperLeftFingerTip = createGripperMeshMarker(
      0.049, 0, -0.05,
      0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_finger_tip_link.dae");
  visualization_msgs::Marker gripperRightFingerTip = createGripperMeshMarker(
      0.049, 0, 0.05,
      -0.707, 0, 0, 0.707,
      "package://robotiq_85_description/meshes/robotiq_85_gripper/visual/robotiq_85_finger_tip_link.dae");

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

  imServer->insert(iMarker);
}

visualization_msgs::Marker Nimbus6dofVis::createGripperMeshMarker(double x, double y, double z, double rx, double ry, double rz, double rw, string meshLocation)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_6dof_vis");

  Nimbus6dofVis n6v;

  ros::spin();
}

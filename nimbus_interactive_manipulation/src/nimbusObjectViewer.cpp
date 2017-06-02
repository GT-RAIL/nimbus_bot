#include <nimbus_interactive_manipulation/nimbusObjectViewer.h>

using namespace std;

NimbusObjectViewer::NimbusObjectViewer()
{
  //messages
  recognizedObjectsSubscriber = n.subscribe("/object_listener/recognized_objects", 1, &NimbusObjectViewer::segmentedObjectsCallback, this);

  imServer.reset(
      new interactive_markers::InteractiveMarkerServer("nimbus_object_viewer", "nimbus_object_markers", false));

  ros::Duration(0.1).sleep();

  imServer->applyChanges();
}

void NimbusObjectViewer::segmentedObjectsCallback(
    const rail_manipulation_msgs::SegmentedObjectList::ConstPtr& objectList)
{
  boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for object list

  //store list of objects
  segmentedObjectList = *objectList;

  ROS_INFO("Received new segmented point clouds");
  clearSegmentedObjects();
  for (unsigned int i = 0; i < objectList->objects.size(); i++)
  {
    if (objectList->objects[i].name == "ignore")
      continue;

    visualization_msgs::InteractiveMarker objectMarker;
    objectMarker.header = objectList->objects[i].marker.header;

    objectMarker.pose.position.x = 0.0;
    objectMarker.pose.position.y = 0.0;
    objectMarker.pose.position.z = 0.0;
    objectMarker.pose.orientation.x = 0.0;
    objectMarker.pose.orientation.y = 0.0;
    objectMarker.pose.orientation.z = 0.0;
    objectMarker.pose.orientation.w = 1.0;

    stringstream ss;
    ss.str("");
    ss << "object" << i;
    objectMarker.name = ss.str();

    visualization_msgs::InteractiveMarkerControl objectControl;
    ss << "control";
    objectControl.name = ss.str();
    objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
    //objectControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    objectControl.always_visible = true;
    objectControl.markers.resize(1);
    //objectControl.markers[0] = cloudMarker;
    objectControl.markers[0] = objectList->objects[i].marker;
    objectMarker.controls.push_back(objectControl);

    //object label
    visualization_msgs::InteractiveMarkerControl objectLabelControl;
    stringstream namestream;
    namestream.str("");
    namestream << "object" << i << "_label";
    objectLabelControl.name = namestream.str();
    objectLabelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
    objectLabelControl.always_visible = true;
    visualization_msgs::Marker objectLabel;
    objectLabel.header = objectList->objects[i].point_cloud.header;
    objectLabel.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    objectLabel.pose.position.x = objectList->objects[i].centroid.x;
    objectLabel.pose.position.y = objectList->objects[i].centroid.y;
    objectLabel.pose.position.z = objectList->objects[i].centroid.z + .1;
    objectLabel.scale.x = .1;
    objectLabel.scale.y = .1;
    objectLabel.scale.z = .1;
    objectLabel.color.r = 1.0;
    objectLabel.color.g = 1.0;
    objectLabel.color.b = 1.0;
    objectLabel.color.a = 1.0;
    objectLabel.text = objectList->objects[i].name;
    objectLabelControl.markers.resize(1);
    objectLabelControl.markers[0] = objectLabel;
    objectMarker.controls.push_back(objectLabelControl);

    //pickup menu
    visualization_msgs::InteractiveMarkerControl objectMenuControl;
    objectMenuControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    ss << "_menu";
    objectMenuControl.name = ss.str();
    objectMarker.controls.push_back(objectMenuControl);

    imServer->insert(objectMarker);

    segmentedObjects.push_back(objectMarker);
  }

  imServer->applyChanges();

  ROS_INFO("Point cloud markers created");
}

void NimbusObjectViewer::clearSegmentedObjects()
{
  boost::recursive_mutex::scoped_lock lock(api_mutex); //lock for object list

  for (unsigned int i = 0; i < segmentedObjects.size(); i++)
  {
    stringstream ss;
    ss.str("");
    ss << "object" << i;
    imServer->erase(ss.str());
  }
  segmentedObjects.clear();

  imServer->applyChanges();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nimbus_object_viewer");

  NimbusObjectViewer nov;

  ros::spin();
}

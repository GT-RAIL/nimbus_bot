#include <nimbus_perception/ObjectLabeler.h>

using namespace std;

ObjectLabeler::ObjectLabeler() : pnh("~")
{
  objectMeasurementPublisher = pnh.advertise<nimbus_perception::TrackedObjectList>("object_measurements", 1);

  recognizeClient = n.serviceClient<nimbus_perception::ClassifyInstance>("simple_recognition/classify_instance");

  newDataSubscriber = n.subscribe<rail_manipulation_msgs::SegmentedObjectList>("rail_segmentation/segmented_objects", 1, &ObjectLabeler::segmentedObjectsCallback, this);
}

void ObjectLabeler::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData)
{
  if (newData->cleared)
    return;

  nimbus_perception::TrackedObjectList measurements;
  measurements.header = newData->header;

  for (unsigned int i = 0; i < newData->objects.size(); i ++)
  {
    measurements.objects.push_back(this->calculateMeasurement(newData->objects[i]));
    measurements.objects[i].name = this->disambiguateSpecialCase(measurements.objects[i]);
  }

  objectMeasurementPublisher.publish(measurements);
}

std::string ObjectLabeler::disambiguateSpecialCase(nimbus_perception::TrackedObject object)
{
  if (object.name == "drawer-handle")
  {
    //TODO: use z coordinate to decide between drawer-handle-top, drawer-handle-middle, and drawer-handle-bottom
    return "drawer-handle-todo";
  }

  return object.name;
}

nimbus_perception::TrackedObject ObjectLabeler::calculateMeasurement(rail_manipulation_msgs::SegmentedObject object)
{
  nimbus_perception::TrackedObject measurement;

  //calculate minimum area bounding box
  //convert point cloud to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(object.point_cloud, converter);
  pcl::fromPCLPointCloud2(converter, *objectCloud);

  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  centroid[0] = object.centroid.x;
  centroid[1] = object.centroid.y;
  centroid[2] = object.centroid.z;
  pcl::computeCovarianceMatrixNormalized(*objectCloud, centroid, covariance);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eig_dx = eigen_solver.eigenvectors();
  eig_dx.col(2) = eig_dx.col(0).cross(eig_dx.col(1));

  //move the points to that reference frame
  Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
  p2w.block(0, 0, 3, 3) = eig_dx.transpose();
  p2w.block(0, 3, 3, 1) = -1.f * (p2w.block(0, 0, 3, 3) * centroid.head(3));
  pcl::PointCloud<pcl::PointXYZRGB> c_points;
  pcl::transformPointCloud(*objectCloud, c_points, p2w);

  pcl::PointXYZRGB min_pt, max_pt;
  pcl::getMinMax3D(c_points, min_pt, max_pt);
  const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

  //final transform
  const Eigen::Quaternionf qfinal(eig_dx);
  const Eigen::Vector3f tfinal = eig_dx * mean_diag + centroid.head(3);

  //approximate shape of object
  measurement.bounds.type = shape_msgs::SolidPrimitive::BOX;
  measurement.bounds.dimensions.resize(3);
  measurement.bounds.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_pt.x - min_pt.x;
  measurement.bounds.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_pt.y - min_pt.y;
  measurement.bounds.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_pt.z - min_pt.z;

  measurement.pose.header.frame_id = object.point_cloud.header.frame_id;
  measurement.pose.header.stamp = object.point_cloud.header.stamp;
  measurement.pose.pose.position.x = tfinal[0];
  measurement.pose.pose.position.y = tfinal[1];
  measurement.pose.pose.position.z = tfinal[2];
  measurement.pose.pose.orientation.w = qfinal.w();
  measurement.pose.pose.orientation.x = qfinal.x();
  measurement.pose.pose.orientation.y = qfinal.y();
  measurement.pose.pose.orientation.z = qfinal.z();

  //sort dims from least to greatest (for classifier)
  vector<float> dims;
  dims.push_back(fabs(max_pt.x - min_pt.x));
  dims.push_back(fabs(max_pt.y - min_pt.y));
  dims.push_back(fabs(max_pt.z - min_pt.z));
  sort(dims.begin(), dims.end());

  //get classification
  nimbus_perception::ClassifyInstance srv;
  srv.request.r = object.marker.color.r;
  srv.request.g = object.marker.color.g;
  srv.request.b = object.marker.color.b;
  srv.request.dims.x = dims[0];
  srv.request.dims.y = dims[1];
  srv.request.dims.z = dims[2];

  if (!recognizeClient.call(srv))
  {
    ROS_INFO("Error calling simple recognition service.");
    measurement.name = "unknown";
  }
  else
  {
    measurement.name = srv.response.label;
  }
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "object_labeler");

  ObjectLabeler ol;

  ros::spin();

  return EXIT_SUCCESS;
}

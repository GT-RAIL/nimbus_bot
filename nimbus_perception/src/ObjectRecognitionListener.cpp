#include <nimbus_perception/ObjectRecognitionListener.h>

using namespace std;

ObjectRecognitionListener::ObjectRecognitionListener() : pnh("~")
{
  // set defaults
  string segmentedObjectsTopic("/segmentation/segmented_objects");

  // grab any parameters we need
  pnh.getParam("segmented_objects_topic", segmentedObjectsTopic);

  segmentedObjectsSubscriber = n.subscribe(segmentedObjectsTopic, 1,
                                           &ObjectRecognitionListener::segmentedObjectsCallback, this);
  recognizedObjectsPublisher = pnh.advertise<rail_manipulation_msgs::SegmentedObjectList>(
      "recognized_objects", 1);

  classifyClient = n.serviceClient<nimbus_perception::Classify>("classify");
}

void ObjectRecognitionListener::segmentedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &objectList)
{
  //lock for the object list
  boost::mutex::scoped_lock lock(objectsMutex);

  ROS_INFO("Received %li segmented objects.", objectList->objects.size());

  // store the list
  objects = *objectList;

  // run recognition
  ROS_INFO("Running recognition...");

  for (size_t i = 0; i < objects.objects.size(); i ++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCLPointCloud2 converter;
    pcl_conversions::toPCL(objects.objects[i].point_cloud, converter);
    pcl::fromPCLPointCloud2(converter, *objectCloud);

    // compute principal direction
    Eigen::Matrix3f covariance;
    Eigen::Vector4f centroid;
    centroid[0] = objects.objects[i].centroid.x;
    centroid[1] = objects.objects[i].centroid.y;
    centroid[2] = objects.objects[i].centroid.z;
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

    //sort from least to greatest
    vector<float> dims;
    dims.push_back(fabs(max_pt.x - min_pt.x));
    dims.push_back(fabs(max_pt.y - min_pt.y));
    dims.push_back(fabs(max_pt.z - min_pt.z));
    sort(dims.begin(), dims.end());

    Eigen::Vector3f rgb;
    rgb[0] = objects.objects[i].marker.color.r;
    rgb[1] = objects.objects[i].marker.color.g;
    rgb[2] = objects.objects[i].marker.color.b;
    Eigen::Vector3f lab = RGB2Lab(rgb);
    lab = RGB2Lab(rgb);

    nimbus_perception::Classify classifyObject;
    classifyObject.request.x = dims[0];
    classifyObject.request.y = dims[1];
    classifyObject.request.z = dims[2];
    classifyObject.request.l = lab[0];
    classifyObject.request.a = lab[1];
    classifyObject.request.b = lab[2];

    if (!classifyClient.call(classifyObject))
    {
      ROS_INFO("Couldn't call classify service!");
      ROS_INFO("Unable to perform object recognition.");
      return;
    }

    objects.objects[i].recognized = true;
    objects.objects[i].name = classifyObject.response.label;
  }

  // check if any recognized models should be combined
  if (objects.objects.size() > 1)
  {
    bool somethingCombined = false;
    for (size_t i = 0; i < objects.objects.size() - 1; i++)
    {
      for (size_t j = i + 1; j < objects.objects.size(); j++)
      {
        // two models can potentially be combined if they are the same object type
        if (objects.objects[i].recognized && objects.objects[j].recognized
            && objects.objects[i].name == objects.objects[j].name)
        {
          double distance = sqrt(pow(objects.objects[i].center.x - objects.objects[j].center.x, 2)
                                 + pow(objects.objects[i].center.y - objects.objects[j].center.y, 2)
                                 + pow(objects.objects[i].center.z - objects.objects[j].center.z, 2));
          if (distance <= SAME_OBJECT_DIST_THRESHOLD)
          {
            rail_manipulation_msgs::SegmentedObject combined;
            this->combineModels(objects.objects[i], objects.objects[j], combined);
            objects.objects[i] = combined;
            objects.objects.erase(objects.objects.begin() + j);
            j--;
            somethingCombined = true;
          }
        }
      }
    }

    if (somethingCombined)
    {
      // re-index marker ids since the object_list_.objects indexing has changed
      for (size_t i = 0; i < objects.objects.size(); i++)
      {
        objects.objects[i].marker.id = i;
      }
    }
  }

  // republish the new list
  recognizedObjectsPublisher.publish(objects);

  ROS_INFO("New recognized objects published.");
}

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab(const Eigen::Vector3f& colorRGB)
{
  // for sRGB   -> CIEXYZ see http://www.easyrgb.com/index.php?X=MATH&H=02#text2
  // for CIEXYZ -> CIELAB see http://www.easyrgb.com/index.php?X=MATH&H=07#text7

  double R, G, B, X, Y, Z;

  R = colorRGB[0];
  G = colorRGB[1];
  B = colorRGB[2];

  // linearize sRGB values
  if (R > 0.04045)
    R = pow ( (R + 0.055) / 1.055, 2.4);
  else
    R = R / 12.92;

  if (G > 0.04045)
    G = pow ( (G + 0.055) / 1.055, 2.4);
  else
    G = G / 12.92;

  if (B > 0.04045)
    B = pow ( (B + 0.055) / 1.055, 2.4);
  else
    B = B / 12.92;

  // postponed:
  //    R *= 100.0;
  //    G *= 100.0;
  //    B *= 100.0;

  // linear sRGB -> CIEXYZ
  X = R * 0.4124 + G * 0.3576 + B * 0.1805;
  Y = R * 0.2126 + G * 0.7152 + B * 0.0722;
  Z = R * 0.0193 + G * 0.1192 + B * 0.9505;

  // *= 100.0 including:
  X /= 0.95047;  //95.047;
  //    Y /= 1;//100.000;
  Z /= 1.08883;  //108.883;

  // CIEXYZ -> CIELAB
  if (X > 0.008856)
    X = pow (X, 1.0 / 3.0);
  else
    X = 7.787 * X + 16.0 / 116.0;

  if (Y > 0.008856)
    Y = pow (Y, 1.0 / 3.0);
  else
    Y = 7.787 * Y + 16.0 / 116.0;

  if (Z > 0.008856)
    Z = pow (Z, 1.0 / 3.0);
  else
    Z = 7.787 * Z + 16.0 / 116.0;

  Eigen::Vector3f colorLab;
  colorLab[0] = static_cast<float> (116.0 * Y - 16.0);
  colorLab[1] = static_cast<float> (500.0 * (X - Y));
  colorLab[2] = static_cast<float> (200.0 * (Y - Z));

  return colorLab;
}

void ObjectRecognitionListener::combineModels(const rail_manipulation_msgs::SegmentedObject &model1,
    const rail_manipulation_msgs::SegmentedObject &model2, rail_manipulation_msgs::SegmentedObject &combined) const
{
  ROS_INFO("Combining two %s models...", model1.name.c_str());

  // set members that won't change
  combined.name = model1.name;
  combined.recognized = model1.recognized;
  // keep the first model ID, as merging them won't make sense
  combined.model_id = model1.model_id;
  // TODO: calculate the merged image, for now it uses just the largest
  combined.image = (model1.image.data.size() > model2.image.data.size()) ? model1.image : model2.image;
  combined.confidence = max(model1.confidence, model2.confidence);

  // combine point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 converter1;
  pcl_conversions::toPCL(model1.point_cloud, converter1);
  pcl::fromPCLPointCloud2(converter1, *cloud1);
  pcl::PCLPointCloud2 converter2;
  pcl_conversions::toPCL(model2.point_cloud, converter2);
  pcl::fromPCLPointCloud2(converter2, *cloud2);
  *combined_cloud = *cloud1 + *cloud2;
  pcl::PCLPointCloud2 converter3;
  pcl::toPCLPointCloud2(*combined_cloud, converter3);
  pcl_conversions::fromPCL(converter3, combined.point_cloud);

  // calculate new point cloud attributes (center, centroid, depth, height, width, orientation)
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*combined_cloud, centroid);
  combined.centroid.x = centroid[0];
  combined.centroid.y = centroid[1];
  combined.centroid.z = centroid[2];

  // calculate the new bounding box
  int x_idx, y_idx, z_idx;
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(*combined_cloud, min_pt, max_pt);
  combined.width = max_pt[0] - min_pt[0];
  combined.depth = max_pt[1] - min_pt[1];
  combined.height = max_pt[2] - min_pt[2];

  // calculate the new center
  combined.center.x = (max_pt[0] + min_pt[0]) / 2.0;
  combined.center.y = (max_pt[1] + min_pt[1]) / 2.0;
  combined.center.z = (max_pt[2] + min_pt[2]) / 2.0;

  // recalculate orientation of combined point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
  // project point cloud onto the xy plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = 0;
  coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(combined_cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_cluster);

  //calculate the Eigen vectors of the projected point cloud's covariance matrix, used to determine orientation
  Eigen::Vector4f projected_centroid;
  Eigen::Matrix3f covariance_matrix;
  pcl::compute3DCentroid(*projected_cluster, projected_centroid);
  pcl::computeCovarianceMatrixNormalized(*projected_cluster, projected_centroid, covariance_matrix);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance_matrix, Eigen::ComputeEigenvectors);
  Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
  eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
  //calculate rotation from eigenvectors
  const Eigen::Quaternionf qfinal(eigen_vectors);

  //convert orientation to a single angle on the 2D plane defined by the segmentation coordinate frame
  tf::Quaternion tf_quat;
  tf_quat.setValue(qfinal.x(), qfinal.y(), qfinal.z(), qfinal.w());
  double r, p, y;
  tf::Matrix3x3 m(tf_quat);
  m.getRPY(r, p, y);
  double angle = r + y;
  while (angle < -M_PI)
  {
    angle += 2 * M_PI;
  }
  while (angle > M_PI)
  {
    angle -= 2 * M_PI;
  }
  combined.orientation = tf::createQuaternionMsgFromYaw(angle);

  // combine the two markers
  combined.marker = model1.marker;
  combined.marker.points.insert(combined.marker.points.end(), model2.marker.points.begin(), model2.marker.points.end());

  // set average RGB
  combined.marker.color.r = ((model1.marker.color.r * model1.marker.points.size()) +
                             (model2.marker.color.r * model2.marker.points.size())) /
                            (model1.marker.points.size() + model2.marker.points.size());
  combined.marker.color.g = ((model1.marker.color.g * model1.marker.points.size()) +
                             (model2.marker.color.g * model2.marker.points.size())) /
                            (model1.marker.points.size() + model2.marker.points.size());
  combined.marker.color.b = ((model1.marker.color.b * model1.marker.points.size()) +
                             (model2.marker.color.b * model2.marker.points.size())) /
                            (model1.marker.points.size() + model2.marker.points.size());
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "object_listener");

  ObjectRecognitionListener listener;

  ros::spin();
}
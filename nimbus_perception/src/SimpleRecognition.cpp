#include <nimbus_perception/SimpleRecognition.h>

using namespace std;

SimpleRecognition::SimpleRecognition() : pnh("~")
{
  string filename = "training_data.yaml";
  pnh.getParam("filename", filename);
  pnh.param("radius_threshold", radiusThreshold, 13.0f);

  readTrainingData(filename);

  classifyServer = pnh.advertiseService("classify_instance", &SimpleRecognition::classifyCallback, this);
}

void SimpleRecognition::readTrainingData(std::string filename)
{
  //prepare point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr readData(new pcl::PointCloud<pcl::PointXYZ>);

  //get data from yaml file
  YAML::Node yamlNode = YAML::LoadFile(filename);
  for (unsigned int i = 0; i < yamlNode.size(); i ++)
  {
    pcl::PointXYZ point;
    Eigen::Vector3f rgb;
    rgb[0] = yamlNode[i]["r"].as<float>();
    rgb[1] = yamlNode[i]["g"].as<float>();
    rgb[2] = yamlNode[i]["b"].as<float>();
    Eigen::Vector3f lab = RGB2Lab(rgb);
    //ROS_INFO("l, a, b : %f, %f, %f", lab[0], lab[1], lab[2]);
    point.x = lab[0];
    point.y = lab[1];
    point.z = lab[2];
    readData->points.push_back(point);
    geometry_msgs::Vector3 dim;
    dim.x = yamlNode[i]["x"].as<float>();
    dim.y = yamlNode[i]["y"].as<float>();
    dim.z = yamlNode[i]["z"].as<float>();
    dims.push_back(dim);
    labels.push_back(yamlNode[i]["label"].as<string>());
  }

  trainingDataPoints = readData;

  //prepare kd tree
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tempTree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  tempTree->setInputCloud(trainingDataPoints);
  kdTree = tempTree;
}

bool SimpleRecognition::classifyCallback(nimbus_perception::ClassifyInstance::Request &req, nimbus_perception::ClassifyInstance::Response &res)
{
  ROS_INFO("-------------------------");
  ROS_INFO("Classifying...");

  Eigen::Vector3f rgb;
  rgb[0] = req.r;
  rgb[1] = req.g;
  rgb[2] = req.b;
  Eigen::Vector3f lab = RGB2Lab(rgb);

  pcl::PointXYZ searchPoint;
  searchPoint.x = lab[0];
  searchPoint.y = lab[1];
  searchPoint.z = lab[2];

  std::vector<int> pointIndices;
  std::vector<float> pointDistances;

  int neighbors = kdTree->radiusSearch(searchPoint, radiusThreshold, pointIndices, pointDistances);
  if (neighbors == 1)
  {
    res.label = labels[pointIndices[0]];
  }
  else if (neighbors > 1)
  {
    //use size to break ties
    double minSizeDiff = numeric_limits<double>::infinity();
    int bestIndex = 0;
    for (unsigned int i = 0; i < neighbors; i ++)
    {
      ROS_INFO("Candidate: %s, at distance %f", labels[pointIndices[i]].c_str(), sqrt(pointDistances[i]));

      int testIndex = pointIndices[i];
      double testSizeDiff = fabs(req.dims.x - dims[testIndex].x) + fabs(req.dims.y - dims[testIndex].y) + fabs(req.dims.z - dims[testIndex].z);
      if (testSizeDiff < minSizeDiff)
      {
        minSizeDiff = testSizeDiff;
        bestIndex = testIndex;
      }
    }
    res.label = labels[bestIndex];
  }
  else
  {
    res.label = "unknown";
  }

  return true;
}

//convert from RGB color space to CIELAB color space, taken and adapted from pcl/registration/gicp6d
Eigen::Vector3f RGB2Lab (const Eigen::Vector3f& colorRGB)
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_recognition");

  SimpleRecognition sr;

  ros::spin();

  return EXIT_SUCCESS;
}

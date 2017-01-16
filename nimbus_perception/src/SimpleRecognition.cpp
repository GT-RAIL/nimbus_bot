#include <nimbus_perception/SimpleRecognition.h>

using namespace std;

SimpleRecognition::SimpleRecognition() : pnh("~")
{
  string filename = "training_data.yaml";
  pnh.getParam("filename", filename);

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
    point.x = yamlNode[i]["r"].as<float>();
    point.y = yamlNode[i]["g"].as<float>();
    point.z = yamlNode[i]["b"].as<float>();
    readData->points.push_back(point);
    sizes.push_back(yamlNode[i]["size"].as<double>());
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

  float radius = 0.075;

  pcl::PointXYZ searchPoint;
  searchPoint.x = req.r;
  searchPoint.y = req.g;
  searchPoint.z = req.b;

  std::vector<int> pointIndices;
  std::vector<float> pointDistances;

  int neighbors = kdTree->radiusSearch(searchPoint, radius, pointIndices, pointDistances);
  if (neighbors == 1)
  {
    res.label = labels[pointIndices[0]];
  }
  else if (neighbors > 1)
  {
    //use size to break ties
    float minSizeDiff = numeric_limits<float>::infinity();
    int bestIndex = 0;
    for (unsigned int i = 0; i < neighbors; i ++)
    {
      ROS_INFO("Candidate: %s, at distance %f", labels[pointIndices[i]].c_str(), pointDistances[i]);

      int testIndex = pointIndices[i];
      float testSizeDiff = fabs(req.size - sizes[testIndex]);
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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_recognition");

  SimpleRecognition sr;

  ros::spin();

  return EXIT_SUCCESS;
}

#ifndef SIMPLE_RECOGNITION_H_
#define SIMPLE_RECOGNITION_H_

#include <nimbus_perception/ClassifyInstance.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

class SimpleRecognition
{

public:

  /**
   * \brief Constructor
   */
  SimpleRecognition();

private:
  void readTrainingData(std::string filename);

  bool classifyCallback(nimbus_perception::ClassifyInstance::Request &req, nimbus_perception::ClassifyInstance::Response &res);

  ros::NodeHandle n, pnh;

  ros::ServiceServer classifyServer;

  YAML::Node yamlNode;

  std::vector<std::string> labels;
  std::vector<float> sizes;
  pcl::PointCloud<pcl::PointXYZ>::Ptr trainingDataPoints;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree;
};

#endif

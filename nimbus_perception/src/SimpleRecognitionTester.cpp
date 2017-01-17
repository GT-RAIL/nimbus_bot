#include <nimbus_perception/SimpleRecognitionTester.h>

using namespace std;

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

SimpleRecognitionTester::SimpleRecognitionTester() : pnh("~")
{
  objects.objects.clear();

  currentObjectPublisher = pnh.advertise<sensor_msgs::PointCloud2>("object", 1);

  recognizeClient = n.serviceClient<nimbus_perception::ClassifyInstance>("simple_recognition/classify_instance");

  newDataSubscriber = n.subscribe<rail_manipulation_msgs::SegmentedObjectList>("rail_segmentation/segmented_objects", 1, &SimpleRecognitionTester::newDataCallback, this);
}

void SimpleRecognitionTester::newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData)
{
  boost::mutex::scoped_lock lock(objectMutex);

  if (newData->cleared)
    return;

  objects = *newData;
  index = 0;
  showObject(index);
}

void SimpleRecognitionTester::showObject(unsigned int index)
{
  //show object point cloud
  currentObjectPublisher.publish(objects.objects[index].point_cloud);

  //calculate minimum area bounding box
  //convert point cloud to pcl point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 converter;
  pcl_conversions::toPCL(objects.objects[index].point_cloud, converter);
  pcl::fromPCLPointCloud2(converter, *objectCloud);

  // compute principal direction
  Eigen::Matrix3f covariance;
  Eigen::Vector4f centroid;
  centroid[0] = objects.objects[index].centroid.x;
  centroid[1] = objects.objects[index].centroid.y;
  centroid[2] = objects.objects[index].centroid.z;
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

  //get classification
  nimbus_perception::ClassifyInstance srv;
  srv.request.r = objects.objects[index].marker.color.r;
  srv.request.g = objects.objects[index].marker.color.g;
  srv.request.b = objects.objects[index].marker.color.b;
  srv.request.dims.x = dims[0];
  srv.request.dims.y = dims[1];
  srv.request.dims.z = dims[2];

  if (!recognizeClient.call(srv))
  {
    ROS_INFO("Error calling simple recognition service.");
  }
  else
  {
    ROS_INFO("Result: %s", srv.response.label.c_str());
  }
}

void SimpleRecognitionTester::loop()
{
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("      Reading from Keyboard      ");
  puts("---------------------------------");
  puts(" Press Q/W to cycle back/forward ");

  while (ros::ok())
  {
    // get the next event from the keyboard
    char c;
    if (read(kfd, &c, 1) < 0)
    {
      ROS_ERROR("Could not read input from keyboard.");
      exit(-1);
    }

    //Display help message
    if (c == KEYCODE_Q)
    {
      boost::mutex::scoped_lock lock(objectMutex);
      if (index != 0)
      {
        index --;
        showObject(index);
      }

    }
    else if (c == KEYCODE_W)
    {
      boost::mutex::scoped_lock lock(objectMutex);
      if (!objects.objects.empty() && index < objects.objects.size() - 1)
      {
        index ++;
        showObject(index);
      }
    }
  }
}

void shutdown(int sig)
{
  // shut everything down
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "simple_recognition_tester");

  // initialize the keyboard controller
  SimpleRecognitionTester srt;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread myThread(boost::bind(&SimpleRecognitionTester::loop, &srt));
  ros::spin();

  // wait for everything to end
  myThread.interrupt();
  myThread.join();
}

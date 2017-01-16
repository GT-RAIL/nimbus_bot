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

  //get classification
  nimbus_perception::ClassifyInstance srv;
  srv.request.r = objects.objects[index].marker.color.r;
  srv.request.g = objects.objects[index].marker.color.g;
  srv.request.b = objects.objects[index].marker.color.b;
  srv.request.size = sqrt(pow(objects.objects[index].width,2) + pow(objects.objects[index].height,2) + pow(objects.objects[index].depth,2));

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

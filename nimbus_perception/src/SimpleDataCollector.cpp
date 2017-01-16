#include <nimbus_perception/SimpleDataCollector.h>

using namespace std;

// used for capturing keyboard input
int kfd = 0;
struct termios cooked, raw;

SimpleDataCollector::SimpleDataCollector() : pnh("~")
{
  objects.objects.clear();

  currentObjectPublisher = pnh.advertise<sensor_msgs::PointCloud2>("object", 1);

  newDataSubscriber = n.subscribe<rail_manipulation_msgs::SegmentedObjectList>("rail_segmentation/segmented_objects", 1, &SimpleDataCollector::newDataCallback, this);
}

void SimpleDataCollector::newDataCallback(const rail_manipulation_msgs::SegmentedObjectList::ConstPtr &newData)
{
  boost::mutex::scoped_lock lock(objectMutex);

  if (newData->cleared)
    return;

  objects = *newData;
  index = 0;
  showObject(index);
}

void SimpleDataCollector::showObject(unsigned int index)
{
  //show object point cloud
  currentObjectPublisher.publish(objects.objects[index].point_cloud);

  //display object info
  ROS_INFO("--------------------------------------------");
  ROS_INFO("Showing data for object %d", index);
  ROS_INFO("RGB: %f, %f, %f", objects.objects[index].marker.color.r, objects.objects[index].marker.color.g, objects.objects[index].marker.color.b);
  ROS_INFO("Bounding Box: %f, %f, %f", objects.objects[index].width, objects.objects[index].height, objects.objects[index].depth);
  ROS_INFO("Center: %f, %f, %f", objects.objects[index].center.x, objects.objects[index].center.y, objects.objects[index].center.z);
  ROS_INFO("Size heuristic: %f", sqrt(pow(objects.objects[index].width,2) + pow(objects.objects[index].height,2) + pow(objects.objects[index].depth,2)));
}

void SimpleDataCollector::loop()
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
  ros::init(argc, argv, "simple_data_collector");

  // initialize the keyboard controller
  SimpleDataCollector sdc;
  ros::NodeHandle n;

  // setup the SIGINT signal for exiting
  signal(SIGINT, shutdown);

  // setup the watchdog and key loop in a thread
  boost::thread myThread(boost::bind(&SimpleDataCollector::loop, &sdc));
  ros::spin();

  // wait for everything to end
  myThread.interrupt();
  myThread.join();
}

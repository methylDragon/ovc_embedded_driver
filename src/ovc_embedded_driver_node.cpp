#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/fill_image.h>
#include <ovc_embedded_driver/vdma_driver.h>

int main(int argc, char **argv)
{
  std::cout << "Hello world" << std::endl;
  VDMADriver vdma;

  ros::init(argc, argv, "ovc_embedded_driver_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  image_transport::Publisher pub = it.advertise("ovc/image", 10);

  unsigned char test_arr[1080*720*3];
  int count = 0;
  sensor_msgs::Image msg;
  while (ros::ok())
  {
    /*
    msg.header.stamp = ros::Time::now();
    msg.height = 720;
    msg.width = 1080;
    msg.encoding = "rgb8";
    msg.step = 1080 * 3;
    msg.data = image;
*/
    sensor_msgs::fillImage(msg, "rgb8", 720, 1080, 1080 * 3, vdma.getImage());
    //sensor_msgs::fillImage(msg, "rgb8", 720, 1080, 1080 * 3, test_arr);
    std::cout << "Got image n. " << count++ << std::endl;
    pub.publish(msg);
  }
}

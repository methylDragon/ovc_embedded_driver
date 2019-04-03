#include <iostream>
#include <vector>
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <topic_tools/shape_shifter.h>
#include <ros/message_traits.h>
#include <sstream>

#include <sensor_msgs/fill_image.h>
#include <ovc_embedded_driver/vdma_driver.h>

#define NUM_CAMERAS 7

const int DMA_DEVICES[NUM_CAMERAS] = {1,2,3,4,5,6,7}; // From hardware

template <typename T>
void SerializeToByteArray(const T& msg, std::vector<uint8_t>& destination_buffer)
{ 
  const uint32_t length = ros::serialization::serializationLength(msg);
  destination_buffer.resize( length );
  //copy into your own buffer 
  ros::serialization::OStream stream(destination_buffer.data(), length);
  ros::serialization::serialize(stream, msg);
}

void publish(int device_num)
{
  ros::NodeHandle nh;

  topic_tools::ShapeShifter shape_shifter;
  shape_shifter.morph(
                  ros::message_traits::MD5Sum<sensor_msgs::Image>::value(),
                  ros::message_traits::DataType<sensor_msgs::Image>::value(),
                  ros::message_traits::Definition<sensor_msgs::Image>::value(),
                  "");
  
  std::vector<uint8_t> buffer;

  std::string topic_name("ovc/image" + std::to_string(device_num));
  ros::Publisher pub = shape_shifter.advertise(nh, topic_name.c_str(), 1);
  sensor_msgs::Image msg;
  msg.data.resize(266*1280*3);
  msg.height = 266*3;
  msg.width = 1280;
  msg.encoding = "mono8";
  msg.step = 1280;

  // VDMA declared here so we can prefill the header, vector type is uint8_t
  size_t image_size = msg.data.size();
  SerializeToByteArray(msg, buffer);
  size_t msg_size = buffer.size();
  VDMADriver vdma(DMA_DEVICES[device_num], device_num, buffer, image_size);

  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    // Fill the message
    unsigned char* image_ptr = vdma.getImage();
    msg.header.stamp = ros::Time::now();
    SerializeToByteArray(msg.header, buffer);
    vdma.setHeader(buffer);
    shape_shifter.assign_data(image_ptr, msg_size);
    pub.publish(shape_shifter);
    ros::Time cur_time = ros::Time::now();
    float interval = (cur_time - prev_time).toSec();
    if (interval > 0.05)
      std::cout << "Frame dropped" << std::endl;
    prev_time = cur_time;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ovc_embedded_driver_node");
  // TODO change to iterative initialization
  std::thread t0(publish,0);
  std::thread t1(publish,1);
  std::thread t2(publish,2);
  std::thread t3(publish,3);
  std::thread t4(publish,4);
  std::thread t5(publish,5);
  std::thread t6(publish,6);
  
  t0.join();
}

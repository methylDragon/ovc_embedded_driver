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
#define FRAME_BASEADDR 0x70000000
#define FRAME_OFFSET 0x400000
#define CAMERA_OFFSET 0x1000000

const int DMA_DEVICES[7] = {1,2,3,4,5,6,7}; // From hardware

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

  int count = 0;
  ros::Time prev_time = ros::Time::now();
  bool message_full = false;
  while (ros::ok())
  {
    // Fill the message
    unsigned char* image_ptr = vdma.getImage();
    msg.header.stamp = ros::Time::now();
    SerializeToByteArray(msg.header, buffer);
    vdma.setHeader(buffer);
    //memcpy(&msg.data[0], image_ptr, 1280*266*3);
    // message_full = true;
    // Publish
    //ros::serialization::OStream stream(buffer.data(), buffer.size());
    // TODO: OPTIMIZE HERE
    //shape_shifter.read(stream);
    shape_shifter.assign_data(image_ptr, msg_size);
    pub.publish(shape_shifter);
    //pub.publish(msg);
    ros::Time cur_time = ros::Time::now();
    float interval = (cur_time - prev_time).toSec();
    if (interval > 0.05)
      std::cout << "Frame dropped" << std::endl;
    //std::cout << interval << std::endl;
    prev_time = cur_time;
  }
}

int main(int argc, char **argv)
{
  std::cout << "Hello world" << std::endl;
  ros::init(argc, argv, "ovc_embedded_driver_node");
  /*
  std::unique_ptr<VDMADriver> vdma[NUM_CAMERAS];
  void* image_pointers[NUM_CAMERAS] = {NULL};
  image_transport::Publisher pubs[NUM_CAMERAS];

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  for (int i=0; i<NUM_CAMERAS; ++i)
  {
    vdma[i] = std::make_unique<VDMADriver>(DMA_DEVICES[i], FRAME_BASEADDR + \
                    i * CAMERA_OFFSET, FRAME_OFFSET);
    std::string topic_name("ovc/image" + std::to_string(i));
    pubs[i] = it.advertise(topic_name.c_str(), 10);
  }



  unsigned char test_arr[1280*266*3] = {0};
  int count = 0;
  sensor_msgs::Image msg[NUM_CAMERAS];
  for (int i=0; i<NUM_CAMERAS; ++i)
  {
    msg[i].data.resize(266*1280*3);
    msg[i].height = 266;
    msg[i].width = 1280;
    msg[i].encoding = "rgb8";
    msg[i].step = 1280 * 3;
  }
  ros::Time prev_time = ros::Time::now();
  */
  std::thread t0(publish,0);
  
  std::thread t1(publish,1);
  std::thread t2(publish,2);
  std::thread t3(publish,3);
  std::thread t4(publish,4);
  std::thread t5(publish,5);
  std::thread t6(publish,6);
  
  t0.join();
  /*
  while (ros::ok())
  {
    
    //std::cout << "Sending image packet" << std::endl;
    for (int i=0; i<NUM_CAMERAS; ++i)
    {
      image_pointers[i] = vdma[i]->getImage();
      msg[i].header.stamp = ros::Time::now();
    }
    //sensor_msgs::fillImage(msg, "rgb8", 266 * vdma[0]->NUM_IMAGES, 1280, 1080 * 3, image_pointers[0]);
    for (int i=0; i<NUM_CAMERAS; ++i)
    {
      //sensor_msgs::fillImage(msg[i], "rgb8", 266 * vdma[i]->NUM_IMAGES, 1280, 1080 * 3, image_pointers[i]);
      //memcpy(&msg[i].data[0], image_pointers[i], 1280*266*3);
      //sensor_msgs::fillImage(msg, "rgb8", 266, 1280, 1080 * 3, test_arr);
      //std::cout << "Got image n. " << count++ << std::endl;
      pubs[i].publish(msg[i]);
    }
    ros::Time new_time = ros::Time::now();
    float interval = (new_time - prev_time).toSec();
    if (interval > 0.05)
      std::cout << "Warning, frame dropped" << std::endl; 
    std::cout << interval << std::endl;
    prev_time = new_time;
  }
  */
}

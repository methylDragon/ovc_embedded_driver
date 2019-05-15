#include <vector>
#include <thread>
#include <memory>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_traits.h>

#include <ovc_embedded_driver/vdma_driver.h>
#include <ovc_embedded_driver/i2c_driver.h>

#define NUM_CAMERAS 3

//const int DMA_DEVICES[NUM_CAMERAS] = {1}; // From hardware
const int DMA_DEVICES[NUM_CAMERAS] = {1,2,3}; // From hardware
const int I2C_DEVICES[NUM_CAMERAS] = {4,0,1}; // Files in /dev/i2c-, must match DMA

const int COLOR_CAMERA_ID = 2; // The only color camera is CAM2

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

  // Init camera
  I2CDriver i2c(I2C_DEVICES[device_num]);

  std::string topic_name("ovc/image" + std::to_string(device_num));
  ros::Publisher pub = shape_shifter.advertise(nh, topic_name.c_str(), 1);
  sensor_msgs::Image msg;
  /*
  msg.data.resize(1488*891);
  msg.height = 891;
  msg.width = 1488;
  msg.encoding = "mono8";
  msg.step = 1488;
  */
  // TODO remove embedded data from image
  msg.data.resize(1280*800);
  msg.height = 800;
  msg.width = 1280;
  if (device_num == COLOR_CAMERA_ID)
    msg.encoding="bayer_grbg8";
  else
    msg.encoding = "mono8";
  msg.step = 1280;

  // VDMA declared here so we can prefill the header, vector type is uint8_t
  size_t image_size = msg.data.size();
  SerializeToByteArray(msg, buffer);
  size_t msg_size = buffer.size();
  VDMADriver vdma(DMA_DEVICES[device_num], device_num, buffer, image_size);
  //i2c.enableTestMode();

  ros::Time prev_time = ros::Time::now();
  FILE *fp = fopen("/home/ubuntu/raw_frames", "wb");
  bool first = true;
  while (ros::ok())
  {
    // Fill the message
    unsigned char* image_ptr = vdma.getImage();
    if (first)
    {
      first = false;
      continue;
    }

    std::cout << "Got frame cam n. " << device_num << " Integration time = " << i2c.getIntegrationTime() << " Current gains = " << i2c.getCurrentGains() << std::endl;
    //fwrite(image_ptr, 1, 1280*800, fp);
    
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
    //i2c.changeTestColor(); 
  }
  fclose(fp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ovc_embedded_driver_node");
  std::unique_ptr<std::thread> threads[NUM_CAMERAS];
  std::unique_ptr<I2CDriver> i2c_devs[NUM_CAMERAS];
  for (int i=0; i<NUM_CAMERAS-2; ++i)
    threads[i] = std::make_unique<std::thread>(publish,i);
  
  threads[0]->join();
}

#include <vector>
#include <thread>
#include <memory>
#include <mutex>
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <topic_tools/shape_shifter.h>
#include <ros/message_traits.h>

#include <ovc_embedded_driver/vdma_driver.h>
#include <ovc_embedded_driver/i2c_driver.h>
#include <ovc_embedded_driver/spi_driver.h>

#define NUM_CAMERAS 3

//const int DMA_DEVICES[NUM_CAMERAS] = {1}; // From hardware
const int DMA_DEVICES[NUM_CAMERAS] = {2,3,4}; // From hardware
const int I2C_DEVICES[NUM_CAMERAS] = {0,1,2}; // Files in /dev/i2c-, must match DMA

const int IMU_SYNC_GPIO = 1; // ID of UIO device used with GPIO for syncing

const int COLOR_CAMERA_ID = 2; // The only color camera is CAM2

ros::Time frame_time;
std::mutex ft_mutex; // Make sure we don't corrupt the frame_time

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
  std::unique_lock<std::mutex> frame_time_mutex(ft_mutex, std::defer_lock);
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
    i2c.controlAnalogGain();
    //fwrite(image_ptr, 1, 1280*800, fp);

    frame_time_mutex.lock();
    msg.header.stamp = ros::Time::now();
    frame_time_mutex.unlock();

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

void publish_imu()
{
  SPIDriver spi(IMU_SYNC_GPIO);
  ros::NodeHandle nh;
  int count = 0;
  std::unique_lock<std::mutex> frame_time_mutex(ft_mutex, std::defer_lock);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/ovc/imu", 37); // 1 frame
  sensor_msgs::Imu imu_msg;
  while (ros::ok())
  {
    IMUReading imu = spi.readSensors();
    // TODO check if delay incurred by SPI transaction is indeed negligible
    ros::Time cur_time = ros::Time::now();
    imu_msg.header.stamp = cur_time;
    imu_msg.angular_velocity.x = imu.g_x;
    imu_msg.angular_velocity.y = imu.g_y;
    imu_msg.angular_velocity.z = imu.g_z;
    imu_msg.linear_acceleration.x = imu.a_x;
    imu_msg.linear_acceleration.y = imu.a_y;
    imu_msg.linear_acceleration.z = imu.a_z;
    if (imu.num_sample == 0)
    {
      // Sample synchronised with frame trigger
      frame_time_mutex.lock();
      frame_time = cur_time;
      frame_time_mutex.unlock();
    }
    ++count;
    if (count == 225)
    {
      std::cout << ros::Time::now() << std::endl;
      std::cout << "1s of imu" << std::endl;
      count = 0;
    }
    imu_pub.publish(imu_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ovc_embedded_driver_node");
  std::unique_ptr<std::thread> threads[NUM_CAMERAS + 1]; // one for IMU
  for (int i=0; i<NUM_CAMERAS; ++i)
    threads[i] = std::make_unique<std::thread>(publish,i);
  threads[NUM_CAMERAS] = std::make_unique<std::thread>(publish_imu);

  threads[NUM_CAMERAS]->join();
}

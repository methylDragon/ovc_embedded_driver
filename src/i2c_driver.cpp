#include <fcntl.h>
#include <string>
#include <iostream>
#include <vector>
#include <sys/ioctl.h>
extern "C"
{
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}
#include <ovc_embedded_driver/i2c_driver.h>

I2CDriver::I2CDriver(int i2c_num)
{
  std::string i2c_filename("/dev/i2c-" + std::to_string(i2c_num));
  i2c_fd = open(i2c_filename.c_str(), O_RDWR);
  if (i2c_fd == -1)
    std::cout << "Couldn't open I2C file " << i2c_filename << std::endl;

  // Set the slave address for the device
  if (ioctl(i2c_fd, I2C_SLAVE, CAMERA_ADDRESS) < 0)
    std::cout << "Couldn't set slave address" << std::endl;
}

void I2CDriver::writeRegAddr(uint16_t reg_addr)
{
  // Need to put the 8 MSB in the command byte to simulate 2 byte address
  uint8_t reg_msb = reg_addr >> 8;
  uint8_t reg_lsb = reg_addr & 0xFF;
  // TODO error recovery
  i2c_smbus_write_byte_data(i2c_fd, reg_msb, reg_lsb);
}

int16_t I2CDriver::readRegister(uint16_t reg_addr, int reg_size)
{
  writeRegAddr(reg_addr);
  int16_t ret_val = 0;
  // Data is returned MSB first
  for (int i=reg_size-1; i>=0; --i)
    ret_val |= i2c_smbus_read_byte(i2c_fd) << (i*8); 
  std::cout << ret_val << std::endl;
  return ret_val;
}

void I2CDriver::writeRegister(uint16_t reg_addr, int val, int reg_size)
{
  //writeRegAddr(reg_addr); 
  std::vector<uint8_t> payload(reg_size + 1); // We need to add one byte for address
  payload[0] = reg_addr & 0xFF;
  for (int i=1; i<=reg_size; ++i)
    payload[i] = val >> (8 * (reg_size - i));
  i2c_smbus_write_i2c_block_data(i2c_fd, reg_addr >> 8, payload.size(), &payload[0]); 
}

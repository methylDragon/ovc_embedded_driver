


class I2CDriver
{
  static constexpr unsigned char CAMERA_ADDRESS = 0x10; // 0x20 from datasheet, LSB ignored

  int i2c_fd;

  int16_t readRegister(uint16_t reg_addr, int reg_size);
  void writeRegister(uint16_t reg_addr, int val, int reg_size);

  void writeRegAddr(uint16_t reg_addr);

public:
  I2CDriver(int i2c_num);


};

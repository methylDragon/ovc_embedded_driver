#include <string>
#include <unordered_map>

struct IMUReading
{
  float a_x, a_y, a_z;
  float g_x, g_y, g_z; 
};

class SPIDriver 
{
  static constexpr size_t BUF_SIZE = 32;
  static constexpr unsigned char MASK_WRITE = 0x00;
  static constexpr unsigned char MASK_READ = 0x80;

  static constexpr unsigned char WHO_AM_I = 0x00;
  static constexpr unsigned char USER_CTRL = 0x03;
  static constexpr unsigned char PWR_MGMT_1 = 0x06;
  static constexpr unsigned char ACCEL_XOUT_H = 0x2D;

  static constexpr unsigned char REG_BANK_SEL = 0x7F;

  static constexpr unsigned char CHIP_ID = 0xEA;

  static constexpr float DEFAULT_ACCEL_SENS = 16384; // LSB / g, TODO m/s?
  static constexpr float DEFAULT_GYRO_SENS = 131; // LSB / dps, TODO rad/s?

  int spi_fd;

  float accel_sens, gyro_sens;

  unsigned char tx_buf[BUF_SIZE], rx_buf[BUF_SIZE];

  void Transmit(size_t tx_len, size_t rx_len);

  void writeRegister(unsigned char addr, unsigned char val);

  unsigned char readRegister(unsigned char addr);
  void burstReadRegister(unsigned char addr, int count);


public:
  SPIDriver();
  IMUReading readSensors();

};

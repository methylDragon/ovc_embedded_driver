#include <fcntl.h>
#include <string>
#include <iostream>
#include <vector>
#include <sys/ioctl.h>
#include <boost/program_options.hpp>
extern "C"
{
  #include <linux/i2c.h>
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
}
#include <ovc_embedded_driver/i2c_driver.h>

namespace po = boost::program_options;

I2CDriver::I2CDriver(int i2c_num)
{
  std::string i2c_filename("/dev/i2c-" + std::to_string(i2c_num));
  i2c_fd = open(i2c_filename.c_str(), O_RDWR);
  if (i2c_fd == -1)
    std::cout << "Couldn't open I2C file " << i2c_filename << std::endl;

  // Set the slave address for the device
  if (ioctl(i2c_fd, I2C_SLAVE, CAMERA_ADDRESS) < 0)
    std::cout << "Couldn't set slave address" << std::endl;
  initRegMap();
  configurePLL(EXTCLK_FREQ, VCO_FREQ, PIXEL_RES);
  configureGPIO();
  configureMIPI();
  //enableTestMode();
  std::cout << "I2C Initialization done" << std::endl;
}

void I2CDriver::programFromFile()
{
  /*
  po::options_description desc("Allowed options");
  std::ifstream ifs("/home/luca/60fps_config", std::ifstream::in); 
  */

}

void I2CDriver::initRegMap()
{
  // Append to the hash map all the registers we need
  reg_map.insert(std::make_pair<std::string, I2CRegister>("RESET_REGISTER", I2CRegister(RESET_REGISTER, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("VT_PIX_CLK_DIV", I2CRegister(VT_PIX_CLK_DIV, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("VT_SYS_CLK_DIV", I2CRegister(VT_SYS_CLK_DIV, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("PRE_PLL_CLK_DIV", I2CRegister(PRE_PLL_CLK_DIV, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("PLL_MULTIPLIER", I2CRegister(PLL_MULTIPLIER, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("OP_PIX_CLK_DIV", I2CRegister(OP_PIX_CLK_DIV, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("OP_SYS_CLK_DIV", I2CRegister(OP_SYS_CLK_DIV, 2)));
  // Image
  reg_map.insert(std::make_pair<std::string, I2CRegister>("Y_ADDR_START", I2CRegister(Y_ADDR_START, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("X_ADDR_START", I2CRegister(X_ADDR_START, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("Y_ADDR_END", I2CRegister(Y_ADDR_END, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("X_ADDR_END", I2CRegister(X_ADDR_END, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("X_ODD_INC", I2CRegister(X_ODD_INC, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("Y_ODD_INC", I2CRegister(Y_ODD_INC, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("FRAME_LENGTH_LINES", I2CRegister(FRAME_LENGTH_LINES, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("LINE_LENGTH_PCK", I2CRegister(LINE_LENGTH_PCK, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("OPERATION_MODE_CTRL", I2CRegister(OPERATION_MODE_CTRL, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("COARSE_INTEGRATION_TIME", I2CRegister(COARSE_INTEGRATION_TIME, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AECTRLREG", I2CRegister(AECTRLREG, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_LUMA_TARGET_REG", I2CRegister(AE_LUMA_TARGET_REG, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_COARSE_INTEGRATION_TIME", I2CRegister(AE_COARSE_INTEGRATION_TIME, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_MAX_EXPOSURE_REG", I2CRegister(AE_MAX_EXPOSURE_REG, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_DAMP_MAX_REG", I2CRegister(AE_DAMP_MAX_REG, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_EG_EXPOSURE_HI", I2CRegister(AE_EG_EXPOSURE_HI, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("AE_EG_EXPOSURE_LO", I2CRegister(AE_EG_EXPOSURE_LO, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("COLAMP_BYPASS", I2CRegister(COLAMP_BYPASS, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("ADC_GAIN_MSB", I2CRegister(ADC_GAIN_MSB, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("ADC_GAIN_LSB", I2CRegister(ADC_GAIN_LSB, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("COLAMP_GAIN", I2CRegister(COLAMP_GAIN, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("CURRENT_GAINS", I2CRegister(CURRENT_GAINS, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("SMIA_TEST", I2CRegister(SMIA_TEST, 2)));
  // MIPI
  reg_map.insert(std::make_pair<std::string, I2CRegister>("FRAME_PREAMBLE", I2CRegister(FRAME_PREAMBLE, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("LINE_PREAMBLE", I2CRegister(LINE_PREAMBLE, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("MIPI_TIMING_0", I2CRegister(MIPI_TIMING_0, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("MIPI_TIMING_1", I2CRegister(MIPI_TIMING_1, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("MIPI_TIMING_2", I2CRegister(MIPI_TIMING_2, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("MIPI_TIMING_3", I2CRegister(MIPI_TIMING_3, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("MIPI_TIMING_4", I2CRegister(MIPI_TIMING_4, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("SERIAL_FORMAT", I2CRegister(SERIAL_FORMAT, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("SERIAL_TEST", I2CRegister(SERIAL_TEST, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("DATA_FORMAT_BITS", I2CRegister(DATA_FORMAT_BITS, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("ANALOG_GAIN", I2CRegister(ANALOG_GAIN, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("DATAPATH_SELECT", I2CRegister(DATAPATH_SELECT, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("READ_MODE", I2CRegister(READ_MODE, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("COMPANDING", I2CRegister(COMPANDING, 2)));
  // Testing
  reg_map.insert(std::make_pair<std::string, I2CRegister>("TEST_PATTERN_MODE", I2CRegister(TEST_PATTERN_MODE, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("TEST_DATA_RED", I2CRegister(TEST_DATA_RED, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("TEST_DATA_GREENR", I2CRegister(TEST_DATA_GREENR, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("TEST_DATA_BLUE", I2CRegister(TEST_DATA_BLUE, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("TEST_DATA_GREENB", I2CRegister(TEST_DATA_GREENB, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("GPI_STATUS", I2CRegister(GPI_STATUS, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("FRAME_STATUS", I2CRegister(FRAME_STATUS, 2)));
  reg_map.insert(std::make_pair<std::string, I2CRegister>("LED_FLASH_CONTROL", I2CRegister(LED_FLASH_CONTROL, 2)));
}

int16_t I2CDriver::readRegister(const std::string& reg_name)
{
  I2CRegister reg(reg_map.at(reg_name));
  i2c_smbus_write_byte_data(i2c_fd, reg.addr >> 8, reg.addr & 0xFF);
  int16_t ret_val = 0;
  // Data is returned MSB first
  for (int i=reg.size-1; i>=0; --i)
    ret_val |= i2c_smbus_read_byte(i2c_fd) << (i*8); 
  return ret_val;
}

void I2CDriver::writeRegister(const std::string& reg_name, int val)
{
  I2CRegister reg(reg_map.at(reg_name));
  std::vector<uint8_t> payload(reg.size + 1); // We need to add one byte for address
  payload[0] = reg.addr & 0xFF;
  for (int i=1; i<=reg.size; ++i)
    payload[i] = val >> (8 * (reg.size - i));
  i2c_smbus_write_i2c_block_data(i2c_fd, reg.addr >> 8, payload.size(), &payload[0]); 
}

void I2CDriver::configurePLL(int input_freq, int target_freq, int pixel_res)
{
  //int pll_mult = 2 * target_freq / input_freq; 
  //std::cout << "PLL mult = " << pll_mult << std::endl;
  // Assuming the multiplier can be high enough and will not overflow (max 255?)
  // TODO documentation ambiguous on the clocks, check...
  writeRegister("PLL_MULTIPLIER", 144);
  // We will not divide the input clock
  writeRegister("PRE_PLL_CLK_DIV", 5);
  // Pixel resolution defines number of clocks per pixel
  writeRegister("VT_PIX_CLK_DIV", 4); // DDR?
  // 1 in documentation
  writeRegister("VT_SYS_CLK_DIV", 2);
  // OP registers have same values
  writeRegister("OP_PIX_CLK_DIV", 8);
  writeRegister("OP_SYS_CLK_DIV", 1);
}

void I2CDriver::configureGPIO()
{
  // Enable input buffers
  int16_t reg_val = readRegister("RESET_REGISTER");
  reg_val |= 1 << 8;
  //reg_val |= 1 << 11; // Force PLL ON (likely unnecessary)
  writeRegister("RESET_REGISTER", reg_val);
  // Enable flash output for debugging purposes
  writeRegister("LED_FLASH_CONTROL", 1 << 8);
}
void I2CDriver::configureMIPI()
{
  // Start with image config
  writeRegister("Y_ADDR_START", 0);
  writeRegister("X_ADDR_START", 4);
  writeRegister("Y_ADDR_END", 799);
  writeRegister("X_ADDR_END", 1283);
  writeRegister("X_ODD_INC", 1);
  writeRegister("Y_ODD_INC", 1);
  writeRegister("OPERATION_MODE_CTRL", 3);
  writeRegister("READ_MODE", 0);
  writeRegister("FRAME_LENGTH_LINES", 874);
  writeRegister("LINE_LENGTH_PCK", 1488);
  writeRegister("COARSE_INTEGRATION_TIME", 873);
  //writeRegister("ANALOG_GAIN", 0x000E); // Set coarse gain to 4x
  uint16_t ae_val = 3;
  // Increase gain
  //ae_val |= 1 << 5;
  // Enable embedded data
  writeRegister("SMIA_TEST", 0x1982); // Enables all
  writeRegister("AECTRLREG", ae_val);
  writeRegister("AE_LUMA_TARGET_REG", 0xA000);
  writeRegister("AE_MAX_EXPOSURE_REG", 873/2); // Originally 0x02A0 
  writeRegister("AE_DAMP_MAX_REG", 0x0110);
  writeRegister("AE_EG_EXPOSURE_HI", 873/2 - 100); 
  writeRegister("AE_EG_EXPOSURE_LO", 100); 
  // Set maximum analog gain to 16
  int16_t reg_val = readRegister("COLAMP_BYPASS");
  reg_val &= 0x00FF;
  reg_val |= 0x0300;
  writeRegister("COLAMP_BYPASS", reg_val); 
  reg_val = readRegister("ADC_GAIN_MSB");
  reg_val &= 0x00FF;
  reg_val |= 0xAA00;
  writeRegister("ADC_GAIN_MSB", reg_val); 
  reg_val = readRegister("ADC_GAIN_LSB");
  reg_val &= 0xFF00;
  reg_val |= 0x0044;
  writeRegister("ADC_GAIN_LSB", reg_val); 
  writeRegister("COLAMP_GAIN", 0xA4AA); 

  // Serializer enabled by default, (reset register bit 12), TODO assert?
  // Change to single lane
  writeRegister("SERIAL_FORMAT", 0x0201);
  writeRegister("COMPANDING", 0);
  // 10 bit output, check precompression?
  writeRegister("DATA_FORMAT_BITS", 0x0808);
  writeRegister("DATAPATH_SELECT", 0x9010);
  writeRegister("FRAME_PREAMBLE", 99);
  writeRegister("LINE_PREAMBLE", 67);
  writeRegister("MIPI_TIMING_0", 7047);
  writeRegister("MIPI_TIMING_1", 8727);
  writeRegister("MIPI_TIMING_2", 16459);
  writeRegister("MIPI_TIMING_3", 521);
  writeRegister("MIPI_TIMING_4", 8);
  // Test for MIPI
  uint16_t test_val = 0;
  /* 
  test_val |= 1 << 8; // Lane 0
  test_val |= 1; // Enable test, even column select
  test_val |= 6 << 4; // low speed square wave
  */
  writeRegister("SERIAL_TEST", test_val);
  // Set sensor to streaming mode (enables it)
  //int16_t reg_val = readRegister("RESET_REGISTER");
  //reg_val |= 1 << 2;
  //reg_val &= ~(1 << 2); // Remove master mode
  //writeRegister("RESET_REGISTER", reg_val);
}

void I2CDriver::enableTestMode()
{
  // Set test pattern mode, 1 = solid color 2 = bars 3 = fade to gray 256 = walking 1
  writeRegister("TEST_DATA_RED", 0x1111);
  writeRegister("TEST_DATA_GREENR", 0x2222);
  writeRegister("TEST_DATA_BLUE", 0x3333);
  writeRegister("TEST_DATA_GREENB", 0x4444);
  writeRegister("TEST_PATTERN_MODE", 1);
  std::cout << "Reset reg = " << std::hex << readRegister("RESET_REGISTER") << std::endl;

  std::cout << "GPI reg = " << std::hex << readRegister("GPI_STATUS") << std::endl;
  std::cout << "FRAME reg = " << std::hex << readRegister("FRAME_STATUS") << std::endl;
  std::cout << "SERIAL_TEST reg = " << std::hex << readRegister("SERIAL_TEST") << std::endl;
}

int16_t I2CDriver::getIntegrationTime() 
{
  return readRegister("AE_COARSE_INTEGRATION_TIME");
}

void I2CDriver::changeTestColor()
{
  static int color=0;
  int red = color == 0 ? 0xFFFF : 0; 
  int green = color == 0 ? 0xFFFF : 0; 
  int blue = color == 0 ? 0xFFFF : 0; 
  writeRegister("TEST_DATA_RED", red);
  writeRegister("TEST_DATA_GREENR", green);
  writeRegister("TEST_DATA_BLUE", blue);
  writeRegister("TEST_DATA_GREENB", green);
  color = (color + 1) % 2;
}

int16_t I2CDriver::getCurrentGains()
{
  return readRegister("CURRENT_GAINS") >> 11; 
}

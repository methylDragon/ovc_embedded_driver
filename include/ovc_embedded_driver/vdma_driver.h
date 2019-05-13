#include <string>
#include <vector>

class VDMADriver
{
  const std::string DMA_FOLDER = "/dev/hugepages/";
  const size_t MAP_SIZE = 1024*1024*2; // Huge page size is 2MB
  // Register addresses
  const int VDMACR = 0x30 / sizeof(int);
  const int VDMASR = 0x34 / sizeof(int);
  const int VSIZE_REG = 0xA0 / sizeof(int);
  const int HSIZE_REG = 0xA4 / sizeof(int);
  const int FRMDLY_STRIDE_REG = 0xA8 / sizeof(int);
  const int PARK_PTR_REG = 0x28 / sizeof(int);
  const int START_ADDR_0 = 0xAC / sizeof(int);

  static constexpr int NUM_FRAMEBUFFERS = 3;

  // Value to write to UIO to reset the interrupt
  const int IRQ_RST = 1;
  // TODO parametrise bits per pixel
  //const size_t SIZEX = 1488;
  //const size_t SIZEY = 891;
  const size_t SIZEX = 1280; // Add rows of embedded data
  const size_t SIZEY = 800;
  const size_t STRIDE = SIZEX;
  const size_t IMAGE_SIZE = SIZEX * SIZEY;
  const size_t UIO_MAP_SIZE = 0x1000;
  // UIO is for AXI4 lite configuration, memory is to access DDR and images
  int uio_file;
  unsigned int *uio_mmap;
  unsigned char *memory_mmap[NUM_FRAMEBUFFERS];
  
  void configureVDMA();
  void startVDMA();
  void setFramebuffer(int i, const std::string& memory_filename);
  void sendFramebuffer(int fb_num, uint32_t address);

  void writeRegister(int reg_addr, int value);

  unsigned int readRegister(int reg_addr) const;

  int frame_offset;
  int misalignment_offset; 

  int last_fb;

public:

  void setHeader(const std::vector<uint8_t>& header, int index = -1);

  VDMADriver(int uio_num, int cam_num, const std::vector<uint8_t>& sample_msg, size_t img_size);
  unsigned char* getImage();
};

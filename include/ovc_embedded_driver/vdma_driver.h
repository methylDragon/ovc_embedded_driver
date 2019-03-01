#include <vector>



class VDMADriver
{
  // Register addresses
  const int VDMACR = 0x30 / sizeof(int);
  const int VDMASR = 0x34 / sizeof(int);
  const int VSIZE = 0xA0 / sizeof(int);
  const int HSIZE = 0xA4 / sizeof(int);
  const int FRMDLY_STRIDE = 0xA8 / sizeof(int);
  const int START_ADDR_0 = 0xAC / sizeof(int);

  // Value to write to UIO to reset the interrupt
  const int IRQ_RST = 1;
  // TODO make framebuffers a vector
  const int MEMORY_MAP_OFFSET = 0x70000000;
  const int FRAMEBUFFER_0 = 0x0;
  // Number of words per image, TODO parametrise channels, bits per pixel, x and y resolutions
  const size_t IMAGE_SIZE = 1080 * 720 * 3;
  // TODO check memory map size, might need larger but must be careful not to overshoot into PMUFW reserved DDR
  //const size_t MEMORY_MAP_SIZE = 0x8000000;
  const size_t MEMORY_MAP_SIZE = 2332800;
  const size_t UIO_MAP_SIZE = 0x1000;
  // UIO is for AXI4 lite configuration, memory is to access DDR and images
  int uio_file, memory_file;
  unsigned int *uio_mmap;
  unsigned char *memory_mmap;
  
  unsigned char dummy_mem[1080*720*3];
  void configureVDMA();

public:
  VDMADriver();

  void* getImage();
};

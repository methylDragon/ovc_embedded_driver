

class VDMADriver
{
  // Register addresses
  const int VDMACR = 0x30 / sizeof(int);
  const int VDMASR = 0x34 / sizeof(int);
  const int VSIZE = 0xA0 / sizeof(int);
  const int HSIZE = 0xA4 / sizeof(int);
  const int FRMDLY_STRIDE = 0xA8 / sizeof(int);
  const int PARK_PTR_REG = 0x28 / sizeof(int);
  const int START_ADDR_0 = 0xAC / sizeof(int);

  static constexpr int NUM_FRAMEBUFFERS = 3;

  // Value to write to UIO to reset the interrupt
  const int IRQ_RST = 1;
  // Number of words per image, TODO parametrise channels, bits per pixel, x and y resolutions and num_images
  const size_t IMAGE_SIZE = 1280 * 266 * 3 * NUM_IMAGES;
  // Memory size chosen not to overshoot the PMU
  const size_t UIO_MAP_SIZE = 0x1000;
  // UIO is for AXI4 lite configuration, memory is to access DDR and images
  int uio_file;
  unsigned int *uio_mmap;
  unsigned char *memory_mmap;
  
  unsigned char dummy_mem[(1280/3)*800*3];
  void configureVDMA();

  int frame_offset;

public:
  const int NUM_IMAGES = 1;

  VDMADriver(int uio_num, int base_addr, int frame_off = 0x400000);
  void* getImage();
};

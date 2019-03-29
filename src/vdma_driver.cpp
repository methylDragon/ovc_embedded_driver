#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <ovc_embedded_driver/vdma_driver.h>



#include <iostream>



VDMADriver::VDMADriver(int uio_num, int base_addr, int frame_off)
{
  // TODO parametrise UIO, done in the Python library...
  std::string uio_filename("/dev/uio" + std::to_string(uio_num));
  uio_file = open(uio_filename.c_str(), O_RDWR);
  int memory_file = open("/dev/mem", O_RDONLY);

  frame_offset = frame_off;

  uio_mmap = (unsigned int*) mmap(NULL, UIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, uio_file, 0);
  memory_mmap = (unsigned char*) mmap(NULL, frame_off * NUM_FRAMEBUFFERS, PROT_READ, MAP_SHARED | MAP_LOCKED, memory_file, base_addr);
  
  configureVDMA();
}


void VDMADriver::configureVDMA()
{
  // TODO implement, for now done separately in Python driver (could actually make it a Python service!)
}

// Return pointer to memory area with image
void* VDMADriver::getImage()
{
  //unsigned char ret_image[IMAGE_SIZE];
  // Return value is ignored, call is blocking until interrupt is generated
  unsigned int dummy;
  *(uio_mmap + VDMASR) = (*(uio_mmap + VDMASR)) | (1 << 12);
  write(uio_file, (char *)&IRQ_RST, sizeof(IRQ_RST));
  unsigned int nb = read(uio_file, &dummy, sizeof(dummy));
  // Reset interrupt
  //std::cout << *(uio_mmap + VDMASR) << std::endl;
  //msync(uio_mmap, UIO_MAP_SIZE, 0);
  // TODO parametrize below
  int fb_num = (*(uio_mmap + PARK_PTR_REG) >> 24) & 0b11111;
  fb_num -= 1;
  if (fb_num < 0) fb_num = 2;
  //std::cout << fb_num << std::endl;
  
  return memory_mmap + fb_num * frame_offset;
  return dummy_mem;
  //memcpy(&ret_image[0], memory_mmap, IMAGE_SIZE); 
}

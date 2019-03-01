#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <ovc_embedded_driver/vdma_driver.h>



#include <iostream>



VDMADriver::VDMADriver()
{
  // TODO parametrise UIO, done in the Python library...
  uio_file = open("/dev/uio1", O_RDWR);
  memory_file = open("/dev/mem", O_RDONLY);

  uio_mmap = (unsigned int*) mmap(NULL, UIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, uio_file, 0);
  memory_mmap = (unsigned char*) mmap(NULL, MEMORY_MAP_SIZE, PROT_READ, MAP_SHARED, memory_file, MEMORY_MAP_OFFSET);
  
  configureVDMA();

  // Needed hack, gets stuck without
  write(uio_file, (char *)&IRQ_RST, sizeof(IRQ_RST));
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
  unsigned int nb = read(uio_file, &dummy, sizeof(dummy));
  // Reset interrupt
  write(uio_file, (char *)&IRQ_RST, sizeof(IRQ_RST));
  *(uio_mmap + VDMASR) = (*(uio_mmap + VDMASR)) | (1 << 12);
  //std::cout << *(uio_mmap + VDMASR) << std::endl;
  msync(uio_mmap, UIO_MAP_SIZE, 0);
  // TODO get actual image from /dev/mem and return it
  // TODO parametrize below
  return memory_mmap;
  return dummy_mem;
  //memcpy(&ret_image[0], memory_mmap, IMAGE_SIZE); 
}

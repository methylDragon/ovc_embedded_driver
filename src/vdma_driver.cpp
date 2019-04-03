#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <string.h>
#include <ovc_embedded_driver/vdma_driver.h>


#include <fstream>
#include <iostream>



VDMADriver::VDMADriver(int uio_num, int cam_num, const std::vector<uint8_t>& sample_msg, size_t img_size)
{
  std::string uio_filename("/dev/uio" + std::to_string(uio_num));
  uio_file = open(uio_filename.c_str(), O_RDWR);
  uio_mmap = (unsigned int*) mmap(NULL, UIO_MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, uio_file, 0);

  size_t header_size = sample_msg.size() - img_size;
  // We want an aligned word for the DMA, however the header is not word aligned.
  // TODO will still be offset by 1 word if header is actually the correct size, fix.
  misalignment_offset = 8 - (header_size % 8);
  frame_offset = header_size + misalignment_offset;
  configureVDMA();
  for (int i=0; i<NUM_FRAMEBUFFERS; ++i)
  {
    // Start by opening (or creating) the camera dma files
    std::string memory_filename("cam" + std::to_string(cam_num) + "_" + std::to_string(i));
    int memory_file = open((DMA_FOLDER + memory_filename).c_str(), O_RDWR | O_CREAT);

    memory_mmap[i] = (unsigned char*) mmap(NULL, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_HUGETLB | MAP_POPULATE, memory_file, 0);
    // Make sure Linux allocates all the pages
    unsigned char dummy[1024*1024*2];
    memcpy((void *)(memory_mmap[i] + misalignment_offset), dummy, sizeof(dummy) - misalignment_offset);
    // Prefill message
    if (sample_msg.size() > MAP_SIZE)
      std::cout << "Error, image size exceeds page size" << std::endl;

    // We copy the whole message, to use as a template, following iterations will only update what is necessary
    // (data through DMA and timestamp through setHeader calls)
    setHeader(sample_msg, i);

    setFramebuffer(i, memory_filename);
  }
  startVDMA();
}

void VDMADriver::setFramebuffer(int i, const std::string& memory_filename)
{
  // Get process id, scrape file that describes mapping virtual to physical memory and map it into the VDMA module
  std::string process_id = std::to_string(getpid());
  std::fstream map_file;
  int pagemap_file = open(("/proc/" + process_id + "/pagemap").c_str(), O_RDONLY);
  map_file.open("/proc/" + process_id + "/maps");
  if (!map_file.is_open() || pagemap_file < 0)
  {
    std::cout << "Error opening mapping file" << std::endl;
    return;
  }
  while (!map_file.eof())
  {
    std::string line;
    std::getline(map_file, line);
    if (line.find(memory_filename, 0) != std::string::npos)
    {
      std::string address_str(line.substr(0,8));
      int memory_address = std::stoi(address_str, 0, 16);
      // Now we need to find the corresponding physical address in the pagemap file
      // TODO remove magic number
      int virtual_page_num = memory_address / 4096;
      uint64_t pagemap_descriptor;
      if (pread(pagemap_file, &pagemap_descriptor, sizeof(pagemap_descriptor), \
                              virtual_page_num * sizeof(pagemap_descriptor)) != sizeof(pagemap_descriptor))
        std::cout << "Read wrong data size from pagemap descriptor" << std::endl;
      // Take physical address from page number * page size
      // Refer to https://www.kernel.org/doc/Documentation/vm/pagemap.txt
      uint32_t phys_addr = (pagemap_descriptor & (((uint64_t)1 << 54) - 1)) * 4096 ;
      // Low level functions that sends the address to the VDMA module
      sendFramebuffer(i, phys_addr);
      break;
    }
  }
}

void VDMADriver::sendFramebuffer(int fb_num, uint32_t address)
{
  // We need to add the offset to make sure we don't overwrite the header
  writeRegister(START_ADDR_0 + fb_num, address + frame_offset);
}

void VDMADriver::writeRegister(int reg_addr, int value)
{
  *(uio_mmap + reg_addr) = value;
};

unsigned int VDMADriver::readRegister(int reg_addr) const
{
  return *(uio_mmap + reg_addr); 
};

void VDMADriver::setHeader(const std::vector<uint8_t>& header, int index)
{
  if (index == -1)
    index = last_fb;
  memcpy((void *)(memory_mmap[index] + misalignment_offset), &header[0], header.size());
}

void VDMADriver::configureVDMA()
{
  // TODO implement, for now done separately in Python driver (could actually make it a Python service!)
  // Run DMA
  writeRegister(VDMACR, readRegister(VDMACR) | 1);
  // Enable frame interrupt
  writeRegister(VDMACR, readRegister(VDMACR) | (1 << 12));
  // TODO check genlock
  // Write stride
  writeRegister(FRMDLY_STRIDE_REG, readRegister(FRMDLY_STRIDE_REG) | STRIDE);
  // Horizontal size
  writeRegister(HSIZE_REG, SIZEX);

}

void VDMADriver::startVDMA()
{
  writeRegister(VSIZE_REG, SIZEY);
}
// Return pointer to memory area with image
unsigned char* VDMADriver::getImage()
{
  // Return value is ignored, call is blocking until interrupt is generated
  unsigned int dummy;
  writeRegister(VDMASR, readRegister(VDMASR) | (1 << 12));
  write(uio_file, (char *)&IRQ_RST, sizeof(IRQ_RST));
  unsigned int nb = read(uio_file, &dummy, sizeof(dummy));

  // TODO parametrize below
  last_fb = (*(uio_mmap + PARK_PTR_REG) >> 24) & 0b11111;
  last_fb -= 1;
  if (last_fb < 0) last_fb = NUM_FRAMEBUFFERS - 1;
  return memory_mmap[last_fb] + misalignment_offset;
}

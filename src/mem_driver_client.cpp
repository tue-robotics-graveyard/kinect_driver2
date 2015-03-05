#include <iostream>

#include <opencv2/opencv.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd_image_header.h"

namespace ipc = boost::interprocess;

int main(int argc, char *argv[])
{

    // Open already created shared memory object.
    ipc::shared_memory_object shm_ = ipc::shared_memory_object(ipc::open_only, "MySharedMemory", ipc::read_write);

    ipc::offset_t size;
    shm_.get_size(size);
    std::cout << "Shared memory size = " << size << " bytes" << std::endl;

    ipc::mapped_region mem_buffer_header(shm_, ipc::read_only, 0, sizeof(BufferHeader));

    BufferHeader& buffer_header = *static_cast<BufferHeader*>(mem_buffer_header.get_address());

    int sequence_nr = 0;

    while(true)
    {
        if (buffer_header.sequence_nr != sequence_nr)
        {
            std::cout << buffer_header.sequence_nr << std::endl;
            sequence_nr = buffer_header.sequence_nr;
        }

        usleep(1000);
    }

    return 0;
}

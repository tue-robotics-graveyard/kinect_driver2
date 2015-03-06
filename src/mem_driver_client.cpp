#include <iostream>

#include <opencv2/opencv.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd_image_header.h"

namespace ipc = boost::interprocess;

int main(int argc, char *argv[])
{

    // Open already created shared memory object.
    ipc::shared_memory_object shm = ipc::shared_memory_object(ipc::open_only, "MySharedMemory", ipc::read_write);

    ipc::offset_t size;
    shm.get_size(size);
    std::cout << "Shared memory size = " << size << " bytes" << std::endl;

    ipc::mapped_region mem_buffer_header(shm, ipc::read_only, 0, sizeof(BufferHeader));
    ipc::mapped_region mem_image_header, mem_image_data;

    BufferHeader& buffer_header = *static_cast<BufferHeader*>(mem_buffer_header.get_address());
    RGBDImageHeader* image_header = 0;

    int sequence_nr = 0;
    uint64_t mem_size = 0;

    while(true)
    {
        if (buffer_header.sequence_nr == sequence_nr)
        {
            usleep(1000);
            continue;
        }

        if (buffer_header.memory_block_size != mem_size)
        {
            mem_size = buffer_header.memory_block_size;

            if (mem_size > 0)
            {
                mem_image_header = ipc::mapped_region(shm, ipc::read_write, sizeof(BufferHeader), sizeof(RGBDImageHeader));
                image_header = static_cast<RGBDImageHeader*>(mem_image_header.get_address());

                mem_image_data = ipc::mapped_region(shm, ipc::read_only, sizeof(BufferHeader) + sizeof(RGBDImageHeader));
            }
            else
            {
                image_header = 0;
            }
        }

        if (!image_header || image_header->num_writers > 0)
        {
//            usleep(1000);
            continue;
        }

        std::cout << buffer_header.sequence_nr << std::endl;

        ++image_header->num_readers;

        uchar* image_data = static_cast<uchar*>(mem_image_data.get_address());

        uint64_t rgb_data_size = image_header->rgb_width * image_header->rgb_height * 3;
        uint64_t depth_data_size = image_header->depth_width * image_header->depth_height * 4;

        cv::Mat rgb(image_header->rgb_height, image_header->rgb_width, CV_8UC3);
        cv::Mat depth(image_header->depth_height, image_header->depth_width, CV_32FC1);

        memcpy(rgb.data, image_data, rgb_data_size);
        memcpy(depth.data, image_data + rgb_data_size, depth_data_size);

        --image_header->num_readers;

        // Check image consistency

        bool ok = true;
        int v = rgb.at<cv::Vec3b>(0)[2];
        for(unsigned int i = 0; i < rgb.cols * rgb.rows; ++i)
        {
            if (rgb.at<cv::Vec3b>(i)[2] != v)
            {
                ok = false;
                break;
            }
        }

        std::cout << "RGB: " << (ok ? "OK" : "NOT OK") << " (v = " << v << ")" << std::endl;

//        cv::imshow("rgb", rgb);
//        cv::imshow("depth", depth / 8);

//        cv::waitKey(1);

        sequence_nr = buffer_header.sequence_nr;
    }

    return 0;
}

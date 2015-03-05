#include <iostream>

#include <opencv2/opencv.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd_image_header.h"

namespace ipc = boost::interprocess;

int main(int argc, char *argv[])
{
    double fx = 368.096588;
    double fy = 368.096588;
    double cx = 261.696594;
    double cy = 202.522202;

    // First time
    // Make sure possibly existing memory with same name is removed
    ipc::shared_memory_object::remove("MySharedMemory");

    //Create a shared memory object.
    ipc::shared_memory_object shm(ipc::create_only, "MySharedMemory", ipc::read_write);

    //Set size
    shm.truncate(sizeof(BufferHeader));

    // Map buffer region
    ipc::mapped_region mem_buffer_header(shm, ipc::read_write, 0, sizeof(BufferHeader));

    ipc::mapped_region mem_image_header, mem_image;

    BufferHeader& buffer_header = *static_cast<BufferHeader*>(mem_buffer_header.get_address());
    buffer_header.memory_block_size = 0;
    buffer_header.sequence_nr = 0;
    buffer_header.data_offset = 0;

    RGBDImageHeader* image_header = 0;
    uchar* image_data = 0;

    uint64_t rgb_data_size = 0;
    uint64_t depth_data_size = 0;
    uint64_t image_data_size = 0;

    while(true)
    {
        cv::Mat rgb(960, 1240, CV_8UC3, cv::Scalar(0, 0, 255));
        cv::Mat depth(480, 640, CV_32FC1, 1.0f);

        if (!image_header || rgb.cols != image_header->rgb_width || rgb.rows != image_header->rgb_height
                || depth.cols != image_header->depth_width || depth.rows != image_header->depth_height)
        {
            rgb_data_size = rgb.cols * rgb.rows * 3;
            depth_data_size = depth.cols * depth.rows * 4;
            image_data_size = rgb_data_size + depth_data_size;

            shm.truncate(sizeof(BufferHeader) + sizeof(RGBDImageHeader) + image_data_size);

            mem_image_header = ipc::mapped_region(shm, ipc::read_write, sizeof(BufferHeader), sizeof(RGBDImageHeader));
            mem_image = ipc::mapped_region(shm, ipc::read_write, sizeof(BufferHeader) + sizeof(RGBDImageHeader));

            image_header = static_cast<RGBDImageHeader*>(mem_image_header.get_address());
            image_data = static_cast<uchar*>(mem_image.get_address());

            buffer_header.memory_block_size = sizeof(RGBDImageHeader) + image_data_size;

            image_header->rgb_width = rgb.cols;
            image_header->rgb_height = rgb.rows;
            image_header->depth_width = rgb.cols;
            image_header->depth_height = rgb.rows;
        }

        if (image_header)
        {
            ++buffer_header.sequence_nr;
            memcpy(image_data, rgb.data, rgb.cols * rgb.rows * 3);
            memcpy(image_data + rgb_data_size, depth.data, depth.cols * depth.rows * 4);
        }

        cv::waitKey(30);

    }

    return 0;
}

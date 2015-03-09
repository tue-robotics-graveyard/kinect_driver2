#include <iostream>

#include <opencv2/opencv.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>

#include "rgbd_image_header.h"

#include <signal.h>

namespace ipc = boost::interprocess;

bool stop = false;

void signalHander(int signal)
{
    stop = true;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHander);

    double fx = 368.096588;
    double fy = 368.096588;
    double cx = 261.696594;
    double cy = 202.522202;

    // First time
    // Make sure possibly existing memory with same name is removed
    ipc::shared_memory_object::remove("MySharedMemory");

    //Create a shared memory object.
    ipc::shared_memory_object shm(ipc::create_only, "MySharedMemory", ipc::read_write);

    cv::Mat rgb(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat depth(480, 640, CV_32FC1, 1.0f);

    uint64_t rgb_data_size = rgb.cols * rgb.rows * 3;
    uint64_t depth_data_size = depth.cols * depth.rows * 4;
    uint64_t image_data_size = rgb_data_size + depth_data_size;

    //Set size
    shm.truncate(sizeof(BufferHeader) + image_data_size);

    // Map buffer region
    ipc::mapped_region mem_buffer_header(shm, ipc::read_write, 0, sizeof(BufferHeader));
    ipc::mapped_region mem_image(shm, ipc::read_write, sizeof(BufferHeader));

    BufferHeader* buffer_header = new (mem_buffer_header.get_address()) BufferHeader;
    buffer_header->sequence_nr = 0;
    buffer_header->message_in = false;

    uchar* image_data = new (mem_image.get_address()) uchar[image_data_size];

    buffer_header->rgb_width = rgb.cols;
    buffer_header->rgb_height = rgb.rows;
    buffer_header->depth_width = depth.cols;
    buffer_header->depth_height = depth.rows;

    double v = 0;
    int c = 0;

    while(!stop)
    {
        rgb = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 255 - c, c));
        depth = cv::Mat(480, 640, CV_32FC1, 1.0f);

        {            
            ipc::scoped_lock<ipc::interprocess_mutex> lock(buffer_header->mutex);

            memcpy(image_data, rgb.data, rgb_data_size);
            memcpy(image_data + rgb_data_size, depth.data, depth_data_size);

            buffer_header->cond_empty.notify_one();
            ++buffer_header->sequence_nr;
        }

        v = v + 0.01;
        if (v > 1.0) v = 0;

        c = 255 - c;

        cv::waitKey(30);
    }

    std::cout << "Removing shared memory" << std::endl;

    ipc::shared_memory_object::remove("MySharedMemory");

    return 0;
}

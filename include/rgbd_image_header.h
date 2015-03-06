#ifndef _RGBD_IMAGE_H_
#define _RGBD_IMAGE_H_

#include <stdint.h>

struct BufferHeader
{
    uint64_t memory_block_size; // size of the shared memory block
    uint64_t sequence_nr;       // sequence number of the image (can be used to check if there is a new image)
    uint64_t data_offset;       // offset after this header of where the image is stored in memory
};

struct RGBDImageHeader
{
    uint32_t num_writers;
    uint32_t num_readers;

    uint32_t rgb_width;         // width of rgb image
    uint32_t rgb_height;        // height of rgb image
    uint32_t depth_width;       // width of depth image
    uint32_t depth_height;      // height of depth image
};

#endif

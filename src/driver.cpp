/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <iostream>

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <rgbd/Server.h>

#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rgbd_transport_server");

    ros::NodeHandle nh_private("~");
    std::string frame_id = "rgbd";
    nh_private.getParam("frame_id", frame_id);

    double fx = 368.096588;
    double fy = 368.096588;
    double cx = 261.696594;
    double cy = 202.522202;
    nh_private.getParam("fx", fx);
    nh_private.getParam("fy", fy);
    nh_private.getParam("cx", cx);
    nh_private.getParam("cy", cy);

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

    if(dev == 0)
    {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return -1;
    }

    libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);
    dev->start();

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    rgbd::Server server;
    server.initialize("rgbd", rgbd::RGB_STORAGE_JPG, rgbd::DEPTH_STORAGE_PNG);

    while(ros::ok())
    {
        listener.waitForNewFrame(frames);
        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat rgb_image(rgb->height, rgb->width, CV_8UC3, rgb->data);
        cv::Mat depth_image = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 1000.0f;

        geo::DepthCamera cam_model;
        cam_model.setFocalLengths(fx, fy);
        cam_model.setOpticalTranslation(0, 0);
        cam_model.setOpticalCenter(cx, cy);

//        cv::Mat rgb_image_small(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
//        cv::resize(rgb_image, rgb_image_small, cv::Size(640,480));

//        cv::Mat depth_image_small(240, 320, CV_32FC1, 0.0f);
//        cv::resize(depth_image, depth_image_small, cv::Size(320, 240));

//        cv::imshow("rgb", rgb_image_small);
//        cv::waitKey(1);

        rgbd::Image image(rgb_image, depth_image, cam_model, frame_id, ros::Time::now().toSec());
        server.send(image);

        listener.release(frames);
    }

    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();

    return 0;
}

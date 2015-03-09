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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rgbd_transport_server");

    ros::NodeHandle nh_private("~");
    std::string frame_id = "rgbd";
    nh_private.getParam("frame_id", frame_id);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Depth intrinsic params

    double depth_fx = 368.096588;
    double depth_fy = 368.096588;
    double depth_cx = 261.696594;
    double depth_cy = 202.522202;

    nh_private.getParam("depth_fx", depth_fx);
    nh_private.getParam("depth_fy", depth_fy);
    nh_private.getParam("depth_cx", depth_cx);
    nh_private.getParam("depth_cy", depth_cy);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // RGB intrinsic params

    double rgb_fx = 1060.707250708333;
    double rgb_fy = 1058.608326305465;
    double rgb_cx = 956.354471815484 + 25;
    double rgb_cy = 518.9784429882449;

    nh_private.getParam("rgb_fx", rgb_fx);
    nh_private.getParam("rgb_fy", rgb_fy);
    nh_private.getParam("rgb_cx", rgb_cx);
    nh_private.getParam("rgb_cy", rgb_cy);


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

        float lens_dist_x = 0.052;
        float lens_dist_y = 0;

        float qx = rgb_fx / depth_fx;
        float sx = rgb_fx * lens_dist_x; // delta in pixels at 1 m distance

        float qy = rgb_fy / depth_fy;
        float sy = rgb_fy * lens_dist_y; // delta in pixels at 1 m distance

//        // Registration
        for(int y = 0; y < depth_image.rows; ++y)
        {
            for(int x = 0; x < depth_image.cols; ++x)
            {
                float z = depth_image.at<float>(y, x);
                if (z == 0 || z > 1.3)
                    continue;

                int x_rgb = qx * ((float)x - depth_cx) + (sx / z) + rgb_cx;
                if (x_rgb < 0 || x_rgb >= rgb_image.cols)
                    continue;

                int y_rgb = qy * ((float)y - depth_cy) + (sy / z) + rgb_cy;
                if (y_rgb < 0 || y_rgb >= rgb_image.rows)
                    continue;

//                std::cout << x << ", " << y << " --> " << x_rgb << ", " << y_rgb << std::endl;

                int c = (z / 10) * 255;
                rgb_image.at<cv::Vec3b>(y_rgb, x_rgb) = cv::Vec3b(c, c, c);
            }
        }

        cv::imshow("rgb", rgb_image);
        cv::imshow("depth", depth_image / 10);
        cv::waitKey(3);

//        cv::Mat rgb_image_small(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
//        cv::resize(rgb_image, rgb_image_small, cv::Size(640,480));

//        cv::Mat depth_image_small(240, 320, CV_32FC1, 0.0f);
//        cv::resize(depth_image, depth_image_small, cv::Size(320, 240));

//        cv::imshow("rgb", rgb_image_small);
//        cv::waitKey(1);

//        geo::DepthCamera cam_model;
//        cam_model.setFocalLengths(depth_fx, depth_fy);
//        cam_model.setOpticalTranslation(0, 0);
//        cam_model.setOpticalCenter(depth_cx, depth_cy);

//        rgbd::Image image(rgb_image, depth_image, cam_model, frame_id, ros::Time::now().toSec());
//        server.send(image);

        listener.release(frames);
    }

    // TODO: restarting ir stream doesn't work!
    // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
    dev->stop();
    dev->close();

    return 0;
}

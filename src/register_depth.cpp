#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char *argv[])
{    
    double fx = 368.096588;
    double fy = 368.096588;
    double cx = 261.696594;
    double cy = 202.522202;

    cv::Mat depth(424, 512, CV_32FC1, 1.0f);
    for(int x = 0; x < depth.cols; x += 20)
    {
        cv::line(depth, cv::Point(x, 0), cv::Point(x, depth.rows - 1), 0.0f);
    }

    cv::imshow("image", depth);
    cv::waitKey();


    cv::Mat cam_matrix = cv::Mat::zeros(3, 3, CV_32FC1);
    cam_matrix.at<float>(0, 0) = fx;
    cam_matrix.at<float>(1, 1) = fy;
    cam_matrix.at<float>(0, 2) = cx;
    cam_matrix.at<float>(1, 2) = cy;
    cam_matrix.at<float>(2, 2) = 1;

    cv::Mat dist_coeff = cv::Mat::zeros(1, 5, CV_32FC1);
    dist_coeff.at<float>(0) = 0.084; // k1
    dist_coeff.at<float>(1) = -0.271; // k2
    dist_coeff.at<float>(2) = 0; // p1
    dist_coeff.at<float>(3) = 0; // p2
    dist_coeff.at<float>(4) = 0.102; // k3

    // 3x3 rectification matrix
    cv::Mat R; // = cv::Mat::eye(3, 3, CV_32FC1);
//    R.at<float>(0, 0) = 1;
//    R.at<float>(1, 1) = 1;
//    R.at<float>(2, 2) = 1;

    cv::Mat new_cam_matrix;

    cv::Mat map1, map2;

    cv::initUndistortRectifyMap(cam_matrix, dist_coeff, R, new_cam_matrix, cv::Size(depth.cols, depth.rows), CV_32FC1, map1, map2);

    cv::Mat depth_new;
    cv::remap(depth, depth_new, map1, map2, CV_INTER_NN); // CV_INTER_LINEAR

    cv::imshow("depth_new", depth_new);
    cv::waitKey();

    return 0;
}

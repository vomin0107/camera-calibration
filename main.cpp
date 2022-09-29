#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>

// dimensions of checkerboard
int CHECKERBOARD[2]{ 6, 9 };

int main()
{
    std::vector<cv::String> images;

//    path of checkerboard images
    std::string image_path = "/home/minn/CLionProjects/camera-calibration/images9x6";

//    get paths
    cv::glob(image_path, images);

    std::cout << "the number of images : " << images.size() << std::endl;
    for (auto loop : images)
    {
        std::cout << loop << std::endl;
    }
    if (images.size() == 0)
        std::cout << "image doesn't exist! \n" << std::endl;

//    3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f> > objp;

//    2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f> > imgp;

//    World coordinates for 3D points
    std::vector<cv::Point3f> objp_w;

    for (int i = 0; i < CHECKERBOARD[1]; i++) // CHECKERBOARD[1] = 6
    {
        for (int j = 0; j < CHECKERBOARD[0]; j++) // CHECKERBOARD[0] = 9
        {
//            size of bix = 30mm
            objp_w.push_back(cv::Point3f(j * 30, i * 30, 0));
        }
    }

    cv::Mat frame, gray;

//    vector to store the pixel coordinates of detected chessboard corners
    std::vector<cv::Point2f> corner_pts; // chessboard corners
    bool success;
//    looping over all the images in the directory
    for (int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

//        finding chessboard corners
        success = cv::findChessboardCorners(gray, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (success)
        {
            cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

//            refining pixel coordinates for given 2d points.
            cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

//            displaying the detected corner points on the chessboard
            cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, success);

            objp.push_back(objp_w);
            imgp.push_back(corner_pts);
        }

        cv::imshow("Image", frame);
        cv::imwrite("../images-calibrated/cornered_"+std::to_string(i)+".jpg", frame);
        cv::waitKey(0);
    }
    cv::destroyAllWindows();

//     cameraMatrix is intrinsic parameters
//     distCoeffs is distortion coefficient
//     R, T is extrinsic parameters
    cv::Mat cameraMatrix, distCoeffs, R, T;

//    get parameters by calibration
    cv::calibrateCamera(objp, imgp, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

    std::cout << "-Camera Parameters-\n";
    std::cout << "CameraMatrix\n" << cameraMatrix << "\n\n";
    std::cout << "DistCoeffs\n" << distCoeffs << "\n\n";
    std::cout << "Rotation Vector\n" << R << "\n\n";
    std::cout << "Translation Vector\n" << T << "\n\n\n";

    cv::Mat undistorted_image;
    cv::Mat result_image;

    for (int i = 0; i < images.size(); i++)
    {
        frame = cv::imread(images[i]);

        cv::undistort(frame, undistorted_image, cameraMatrix, distCoeffs); // undistort with parameters
        cv::hconcat(frame, undistorted_image, undistorted_image);
        cv::resize(undistorted_image, undistorted_image,cv::Size( undistorted_image.cols*0.5, undistorted_image.rows*0.5));
        cv::imshow("undistorted_image",undistorted_image);
        cv::imwrite("../images-calibrated/undistorted_"+std::to_string(i)+=".jpg", undistorted_image);

        cv::waitKey(0);
    }
    cv::destroyAllWindows();

    return 0;
}
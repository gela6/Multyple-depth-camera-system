//---------------------------------------- INCLUDE LIBRARIES ----------------------------------------
#include <iostream>
#include <fstream>
#include <filesystem>
#include <experimental/filesystem> //std::experimental::filesystem::create_directory("sand/wand");

#include "open3d/Open3D.h"

#include <Eigen/StdVector>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <string>
#include <cstdlib>
#include <cstring>

#include <stdlib.h>
#include <stdio.h>
#include <chrono>   //   std::this_thread::sleep_for(std::chrono::microseconds(5000));
#include <thread>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
//#include <QCoreApplication>
//#include <QDebug>
//#include <QDir>
//#include <QString>
//#include <iostream>
//#include <fstream>
//#include <cstdlib>
//#include <filesystem>

//#include <Cuda/Open3DCuda.h>

//-----------------------------Opencv-----------------------------
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp> //tracking
#include "opencv2/highgui.hpp"   //openCV module for enableing a Graphic User Interface
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc.hpp>   //image processing algorithms
#include <opencv2/imgcodecs.hpp>   //reading and writing image files
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>   //reconstruct 3D scene from 2D images
#include <vector>
#include <numeric>
#include "opencv2/opencv_modules.hpp"   //cuda video writter
#include <opencv2/imgcodecs.hpp>
//#include <opencv2/gapi.hpp>
//#include <opencv2/gapi/core.hpp>
//#include <opencv2/gapi/imgproc.hpp>   //image processing operations
//#include <opencv2/tracking.hpp> //tracking
//#include <opencv2/ximgproc/segmentation.hpp>
//#include <opencv2/tracking/tracking_legacy.hpp>   // for multitracker
//#include <opencv2/ximgproc.hpp>   //for Perona-Malik anisotropic diffusion and edgePreservingFilter  ,    advanced image processing algorithms
//#include "opencv2/cudacodec.hpp"   //cuda video writter
//---------------------------------------- KRAJ INCLUDE ----------------------------------------


using namespace cv;    //namespace for opencv
using namespace std;
//------------------- Global variables -------------------
int cede = 1;
Mat input1, result1, input2, result2, input3, result3;
//---------
Mat cameraMatrix1, cameraMatrix2, cameraMatrix3; // = (Mat1d(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
Mat distortionCoefficients1, distortionCoefficients2, distortionCoefficients3; // = (Mat1d(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
//----- Matrice i vektori za stereoCalibrate funkciju -----
Mat R, F, E;
Vec3d T;
Size patternSize(7,9);
const float squareSize = 20;
int boardHeight = 9, boardWidth = 7;
int nrCam = 0;
int nrPic = 0;
int brojac = 0;
int low_r=75, low_g=100, low_b=16;
int high_r=255, high_g=255, high_b=36;
//----- Matrix i vectors for stereoCalibrate function -----
//------------------- Global variables -------------------
//------------------- Realsense functions -------------------
// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> fGetTexcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
open3d::geometry::PointCloud fPointsToPc(const rs2::points& points, const rs2::video_frame& color);
std::tuple<open3d::geometry::PointCloud, cv::Mat> fRealsenseCloudGrabber1();
std::tuple<open3d::geometry::PointCloud, cv::Mat> fRealsenseCloudGrabber2();
std::tuple<open3d::geometry::PointCloud, cv::Mat> fRealsenseCloudGrabber3();
cv::Mat fRealsenseImageGrabber1();
cv::Mat fRealsenseImageGrabber2();
cv::Mat fRealsenseImageGrabber3();
//------------------- Realsense functions -------------------
//------------------- Open3D variables -------------------
//auto point_cloud = open3d::geometry::PointCloud();
std::vector<open3d::geometry::PointCloud> oblaci;
std::vector<cv::Mat> slike;
open3d::geometry::PointCloud cloud1, cloud2, cloud3, cloud4, cloud5, cloud6, cloud12, cloud13;
std::tuple< open3d::geometry::PointCloud, cv::Mat> realsenseData1, realsenseData2, realsenseData3;
open3d::visualization::Visualizer vis1, vis2, visM1, visM2;
Eigen::Matrix4d transformMatrix12, transformMatrix13;
//------------------- Open3D variables -------------------
//-------------- Buttons --------------
void bStartCameras(int, void*);
void bStopCallback(int, void*);
void bGetNewImage(int, void*);
void bGetNewCloud(int, void*);
void bSingleCamLive(int, void*);
void bSingleCamSave(int, void*);
void bCloudTransform12(int,void*);
void bCloudTransform13(int,void*);
void bCloudTransform123(int,void*);
// void bCloudLiveStream(int, void*);
void bLiveStereo12(int, void*);
void bLiveStereo13(int, void*);
void bLiveStereo123(int, void*);
void bStereoSavePic(int, void*);
void bFindStereoCalibration(int, void*);
// void bPomakZ(int, void*);
//void bOpen3dVisualizer(int, void*);
void bCloudCleaning(int, void*);
void bsavepics(int, void*);
void bsavecloud(int, void*);
void bsaveframes(int,void*);
void breadcloud(int,void*);
void bpoksuho(int, void*);
void bpokall(int, void*);
void b3T(int, void*);
void bzuto(int, void*);
void bdipl(int, void*);
void bsingle_cloud_icp(int,void*);
void bcede(int,void*);


void on_low_r_thresh_trackbar(int, void *);
void on_high_r_thresh_trackbar(int, void *);
void on_low_g_thresh_trackbar(int, void *);
void on_high_g_thresh_trackbar(int, void *);
void on_low_b_thresh_trackbar(int, void *);
void on_high_b_thresh_trackbar(int, void *);
//-------------- Buttons --------------
//-------------- Functions --------------
void fInitializeDevices();
void fInitializeGUI();
void fApplyAlgorithms();
//void fApplyLiveAlgorithms();
void fDrawAxisAndCorners(Mat img, vector<Point2f> corners, vector< Point3f > obj, string izlaz, bool found);
std::tuple<cv::Mat,cv::Mat, double> fSingleCamCalibration();
//-------------- Functions --------------
//-------------- Realsense global ----------------------
vector<rs2::frameset> proba1, proba2, proba3;    // da probamo
vector<rs2::depth_frame> pokusaj1;
vector<rs2::video_frame> pokusaj2, pokusaj3, pokusaj4, pokusaj5;
//std::array<rs2::video_frame,100> polje;
rs2::pipeline_profile profile1, profile2, profile3;
rs2::frameset frames1, frames2, frames3;
// Declare pointcloud object, for calculating pointclouds and texture mappings
rs2::pointcloud pc1, pc2, pc3;
// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline pipeReal1, pipeReal2, pipeReal3;   //   simplifies streaming flow
//Create a configuration for configuring the pipeline with a non default profile
rs2::config cfg1, cfg2, cfg3;
rs2::align align_to_depth1(RS2_STREAM_DEPTH), align_to_depth2(RS2_STREAM_DEPTH), align_to_depth3(RS2_STREAM_DEPTH);
//-------------- Realsense global ----------------------
//-------------- Affine3f global --------------
Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
//-------------- Affine3f global --------------
/******************************************** The beginning of code ********************************************/
//----------------------------------- Inicialization -----------------------------------
void fInitializeDevices()
{
    rs2::context ctx;
    auto devices = ctx.query_devices();
    size_t deviceCount = devices.size();
    cout << "Loaded devices count: " << deviceCount << endl << endl ;

    // Get the first connected device
    auto dev = devices[0];
    auto serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "Serial number 1: " << serial << endl ;

    // Get the second connected device
    auto dev1 = devices[1];
    auto serial1 = dev1.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "Serial number 2: " << serial1 << endl ;

    //Get the third connected device
    auto dev2 = devices[2];
    auto serial2 = dev2.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    cout << "Serial number 3: " << serial2 << endl << endl;


    // ************* REALSENSE INIT******************
    cfg1.enable_device("018322071221");   //   052622073952   018322071221  102422074003
    cfg1.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);   //640, 480    1280, 720
    cfg1.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg1.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);   //640, 480     1280 × 720


    cfg2.enable_device("052622073952");
    cfg2.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);    //
    cfg2.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg2.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);


    cfg3.enable_device("102422074003");
    cfg3.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);    //
    cfg3.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg3.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    profile1 = pipeReal1.start(cfg1);
    profile2 = pipeReal2.start(cfg2);
    profile3 = pipeReal3.start(cfg3);

    auto frames01 = pipeReal1.wait_for_frames();
    auto frames02 = pipeReal2.wait_for_frames();
    auto frames03 = pipeReal3.wait_for_frames();

    //*************************************** Intrinzične matrice i matrice koeficijenata
    rs2::video_stream_profile video_stream1=profile1.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    if (video_stream1 == nullptr) std::cout << "stream profile is null\n";
    try
    {
        //If the stream is indeed a video stream, we can now simply call get_intrinsics()
        rs2_intrinsics intrinsics = video_stream1.get_intrinsics();
        auto principalPoint = std::make_pair(intrinsics.ppx, intrinsics.ppy);
        auto focalLength = std::make_pair(intrinsics.fx, intrinsics.fy);
        rs2_distortion model = intrinsics.model;

        cameraMatrix1 = (Mat1d(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
        distortionCoefficients1 = (Mat1d(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
        cout<<"Camera matrix1:"<<endl<<cameraMatrix1<<endl;
        cout<<"Distortion Coefficientsx1:"<<distortionCoefficients1<<endl<<endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
    }

    rs2::video_stream_profile video_stream2=profile2.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    if (video_stream2 == nullptr) std::cout << "stream profile is null\n";
    try
    {
        //If the stream is indeed a video stream, we can now simply call get_intrinsics()
        rs2_intrinsics intrinsics = video_stream2.get_intrinsics();
        auto principalPoint = std::make_pair(intrinsics.ppx, intrinsics.ppy);
        auto focalLength = std::make_pair(intrinsics.fx, intrinsics.fy);
        rs2_distortion model = intrinsics.model;

        cameraMatrix2 = (Mat1d(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
        distortionCoefficients2 = (Mat1d(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
        cout<<"Camera matrix2:"<<endl<<cameraMatrix2<<endl;
        cout<<"Distortion Coefficientsx2:"<<distortionCoefficients2<<endl<<endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << endl;
    }

    rs2::video_stream_profile video_stream3=profile3.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    if (video_stream3 == nullptr) std::cout << "stream profile is null\n";
    try
    {
        //If the stream is indeed a video stream, we can now simply call get_intrinsics()
        rs2_intrinsics intrinsics = video_stream3.get_intrinsics();
        auto principalPoint = std::make_pair(intrinsics.ppx, intrinsics.ppy);
        auto focalLength = std::make_pair(intrinsics.fx, intrinsics.fy);
        rs2_distortion model = intrinsics.model;

        cameraMatrix3 = (Mat1d(3, 3) << intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
        distortionCoefficients3 = (Mat1d(1, 5) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3], intrinsics.coeffs[4]);
        cout<<"Camera matrix3:"<<endl<<cameraMatrix3<<endl;
        cout<<"Distortion Coefficientsx3:"<<distortionCoefficients3<<endl<<endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get intrinsics for the given stream. " << e.what() << std::endl;
    }


    //     pipeReal1.stop();
    //     pipeReal2.stop();
    //     pipeReal3.stop();
}
//----------------------------------- Inicialization -----------------------------------
//----------------------------------- Cloud grabber ----------------------------------
std::tuple<open3d::geometry::PointCloud, cv::Mat>   fRealsenseCloudGrabber1()
{

    frames1 = pipeReal1.wait_for_frames();  // ovo ko neka konfiguracija, nebitni su frames01 i 02


    rs2::video_frame coloredFrame = frames1.get_color_frame();
    frames1 = align_to_depth1.process(frames1);
    //    auto depth = frames1.get_depth_frame();
    rs2::depth_frame depth = frames1.get_depth_frame();
    rs2::threshold_filter thresholdFilter;
    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.3f);
    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);
    depth = thresholdFilter.process(depth);

    //    auto coloredFrame = frames1.get_color_frame();

    const int w = coloredFrame.get_width();    //   da bi dobili dimenzije matrice?
    const int h = coloredFrame.get_height();
    cv::Mat image2(cv::Size(w, h), CV_8UC3, (void*)coloredFrame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat image;
    image2.copyTo(image);
    pc1.map_to(coloredFrame);
    //    auto points = pc1.calculate(depth);
    rs2::points points = pc1.calculate(depth);

    open3d::geometry::PointCloud tmpPointCloud = open3d::geometry::PointCloud(fPointsToPc(points, coloredFrame));

    //    pipeReal1.stop();

    return std::make_tuple(tmpPointCloud, image);
}
std::tuple<open3d::geometry::PointCloud, cv::Mat>   fRealsenseCloudGrabber2()
{

    frames2 = pipeReal2.wait_for_frames();  // ovo ko neka konfiguracija, nebitni su frames01 i 02


    rs2::video_frame coloredFrame = frames2.get_color_frame();
    frames2 = align_to_depth2.process(frames2);

    rs2::depth_frame depth = frames2.get_depth_frame();
    rs2::threshold_filter thresholdFilter;
    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.3f);
    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);
    depth = thresholdFilter.process(depth);

    const int w = coloredFrame.get_width();
    const int h = coloredFrame.get_height();
    cv::Mat image2(cv::Size(w, h), CV_8UC3, (void*)coloredFrame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat image;
    image2.copyTo(image);
    pc2.map_to(coloredFrame);
    rs2::points points = pc2.calculate(depth);

    open3d::geometry::PointCloud tmpPointCloud = open3d::geometry::PointCloud(fPointsToPc(points, coloredFrame));

    //    pipeReal2.stop();

    return std::make_tuple(tmpPointCloud, image);
}
std::tuple<open3d::geometry::PointCloud, cv::Mat>   fRealsenseCloudGrabber3()
{

    frames3 = pipeReal3.wait_for_frames();  // ovo ko neka konfiguracija, nebitni su frames01 i 02

    rs2::video_frame coloredFrame = frames3.get_color_frame();
    frames3 = align_to_depth3.process(frames3);

    rs2::depth_frame depth = frames3.get_depth_frame();
    rs2::threshold_filter thresholdFilter;
    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.3f);
    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);
    depth = thresholdFilter.process(depth);

    const int w = coloredFrame.get_width();
    const int h = coloredFrame.get_height();
    cv::Mat image2(cv::Size(w, h), CV_8UC3, (void*)coloredFrame.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat image;
    image2.copyTo(image);
    pc3.map_to(coloredFrame);
    rs2::points points = pc3.calculate(depth);

    open3d::geometry::PointCloud tmpPointCloud = open3d::geometry::PointCloud(fPointsToPc(points, coloredFrame));

    //    pipeReal3.stop();

    return std::make_tuple(tmpPointCloud, image);
}
//----------------------------------- Cloud grabber ----------------------------------
//----------------------------------- Image grabber ----------------------------------
cv::Mat fRealsenseImageGrabber1()   //
{
    //    pipeReal1.start(cfg1);
    rs2::frameset frames = pipeReal1.wait_for_frames();
    rs2::video_frame colorFrame=frames.get_color_frame();
    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    return colorImage;

}
cv::Mat fRealsenseImageGrabber2()   //
{
    //    pipeReal2.start(cfg2);
    rs2::frameset frames = pipeReal2.wait_for_frames();
    rs2::video_frame colorFrame=frames.get_color_frame();
    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    return colorImage;
    //    pipeReal2.stop();
}
cv::Mat fRealsenseImageGrabber3()   //
{
    //    pipeReal3.start(cfg3);
    rs2::frameset frames = pipeReal3.wait_for_frames();
    rs2::video_frame colorFrame = frames.get_color_frame();
    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    return colorImage;
    //    pipeReal3.stop();
}
//----------------------------------- Image grabber ----------------------------------
//----------------------------------- Points and color of a cloud ----------------------------------
// Get RGB values based on normals - texcoords, normals value [u v]
std::tuple<uint8_t, uint8_t, uint8_t> fGetTexcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)  // 300000 puta za svaki point cloud se vrti ovo
{
    const int w = texture.get_width(), h = texture.get_height();   //   t = 3.7e-08 s  u svakoj iteraciji a ima ih dosta

    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u * w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v * h + .5f), 0), h - 1);   //   t = 1.6e-08 s

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto textureData = reinterpret_cast<const uint8_t*>(texture.get_data());   //   t = 6.6e-08 s

    return std::tuple<uint8_t, uint8_t, uint8_t>(textureData[idx], textureData[idx + 1], textureData[idx + 2]);
}
open3d::geometry::PointCloud fPointsToPc(const rs2::points& points, const rs2::video_frame& color)
{
    // OpenCV Mat for showing the rgb color image, just as part of processing
    //    cv::Mat colorr(cv::Size(640, 480), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    rs2::stream_profile sp = points.get_profile().as<rs2::video_stream_profile>();
    auto texCoords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();   //   t = 1.334e-06 s

    std::vector<Eigen::Vector3d> xyzPoints = std::vector<Eigen::Vector3d>();
    std::vector<Eigen::Vector3d> rgbPoints = std::vector<Eigen::Vector3d>();   //   t = 3.3e-08 s
    // Iterating through all points and setting XYZ coordinates
    // and RGB values

    for (int i = 0; i < points.size(); ++i)   //   t = 0.0325059 s
    {

        Eigen::Vector3d tmpVar1 = Eigen::Vector3d(vertices[i].x, vertices[i].y, vertices[i].z);
        xyzPoints.push_back(tmpVar1);                                             //   t = 0.0102909 s  vrijeme potrebno za jedan cloud

        std::tuple<uint8_t, uint8_t, uint8_t> currentColor;   //  možda tu možemo bitove staviti da budu unutar 0-1 , sada su 0-255
        currentColor = fGetTexcolor(color, texCoords[i]);   //   t = 0.0248817 s   !!!! tu je usko grlo

        Eigen::Vector3d tmpVar2 = Eigen::Vector3d(std::get<2>(currentColor)*0.00392156862, std::get<1>(currentColor)*0.00392156862, std::get<0>(currentColor)*0.00392156862);  // t = 0.00495678 s
        rgbPoints.push_back(tmpVar2);      //   t = 0.00929714 s

    }

    //    auto tmpPointCloud = open3d::geometry::PointCloud();       //// KAJ NE MOŽE BEZ AUTO? ovako:   open3d::geometry::PointCloud cloud1
    open3d::geometry::PointCloud tmpPointCloud;
    tmpPointCloud.points_ = xyzPoints;
    tmpPointCloud.colors_ = rgbPoints;   //   t = 0.00471759 s



//    std::shared_ptr<open3d::geometry::RGBDImage> cloudPtr3;
//    open3d::geometry::Image colorss, depthh;
//    open3d::geometry::RGBDImage imagergbd;

//    imagergbd.CreateFromColorAndDepth(colorss, depthh);
//    cloudPtr3 = std::make_shared<open3d::geometry::RGBDImage>(imagergbd);
//    open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan");



    return tmpPointCloud;
}
//----------------------------------- Points and color of a cloud ----------------------------------
//----------------------------------- Buttons functions ----------------------------------
//void bStartCameras(int, void*)   //   Kada stisnemo 'start camera' ova funkcija uzima cloud
//{
//    input1.release();
//    input1 = input01.clone();
//    input2.release();
//    input2 = input02.clone();
//    input3.release();
//    input3 = input03.clone();
//    fApplyAlgorithms();
//}
void bStopCallback(int, void*)   //   shut down
{
    pipeReal1.stop();
    pipeReal2.stop();
    pipeReal3.stop();
    result1.release();
    result2.release();
    result3.release();
    destroyAllWindows();   //remove all windows
    exit(0);
}
void bGetNewCloud(int, void*)   //   uslikaj novi cloud
{
    cloud1.Clear();
    realsenseData1 = fRealsenseCloudGrabber1();
    cloud1 = open3d::geometry::PointCloud(std::get<0>(realsenseData1));
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    cloudPtr1 = cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
//    open3d::io::WritePointCloudToPLY("zuti_cloud1.ply", *cloudPtr1, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    auto input01 = std::get<1>(realsenseData1);// get<1> is mat opencv


    cloud2.Clear();
    realsenseData2 = fRealsenseCloudGrabber2();
    cloud2 = open3d::geometry::PointCloud(std::get<0>(realsenseData2));
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
//    cloudPtr2 = cloud2.VoxelDownSample(0.002);   //  t = 0.00780749 s
//    open3d::io::WritePointCloudToPLY("zuti_cloud2.ply", *cloudPtr2, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    auto input02 = std::get<1>(realsenseData2);// get<1> is mat opencv

    cloud3.Clear();
    realsenseData3 = fRealsenseCloudGrabber3();
    cloud3 = open3d::geometry::PointCloud(std::get<0>(realsenseData3));
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
//    cloudPtr3 = cloud3.VoxelDownSample(0.002);   //  t = 0.00780749 s
//    open3d::io::WritePointCloudToPLY("zuti_cloud3.ply", *cloudPtr3, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    auto input03 = std::get<1>(realsenseData3);// get<1> is mat opencv

    input1.release();
    input1 = input01.clone();
    input2.release();
    input2 = input02.clone();
    input3.release();
    input3 = input03.clone();

    fApplyAlgorithms();
}
void bGetNewImage(int, void*)   //da kamera uslika novu sliku
{
    //    auto input2 = Realsense_Image_Grabber();// get<1> is mat opencv
    //    input.release();
    //    input = input2.clone();
    //    fApplyAlgorithms();

    input1.release();
    input1 = fRealsenseImageGrabber1();// get<1> is mat opencv
//    imwrite("cam1_zuto.png", input1);

    input2.release();
    input2 = fRealsenseImageGrabber2();// get<1> is mat opencv
//    imwrite("cam2_zuto.png", input2);

    input3.release();
    input3 = fRealsenseImageGrabber3();// get<1> is mat opencv
//    imwrite("cam3_zuto.png", input3);

    fApplyAlgorithms();
}
//void bCloudLiveStream(int,void*)
//{
//    while (waitKey(33) != 27)
//    {
//        cloud1->clear();
//        realsenseData1 = fRealsenseCloudGrabber1();
//        cloud1 = std::get<0>(realsenseData1);//point cloud container
//        auto input01 = std::get<1>(realsenseData1);// get<1> is mat opencv
//        cloud2->clear();
//        realsenseData2 = fRealsenseCloudGrabber2();
//        cloud2 = std::get<0>(realsenseData2);//point cloud container
//        auto input02 = std::get<1>(realsenseData2);// get<1> is mat opencv
//        fApplyLiveAlgorithms();
//    }
//}
void bCloudTransform12(int,void*)
{
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.0232;
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -4.05125; front[2] = -4.33795;   // drugi parametar pomiče gore dolje, što veći minus do je više, i treći se nešto može povećavati
    zoom = 0.0212;   // i zoom mi približava i udaljava
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "NACRT", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);


    open3d::io::ReadPointCloud("cloud_nr1.ply", cloud1);
    open3d::io::ReadPointCloud("cloud_nr2.ply", cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    //    cloud1.PaintUniformColor({1, 0.706, 0});
    //    cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr1});



    //***** OVDJE KOPIRATI cloud12 = cloud2, pa dalje s cloud12 raditi tako da cloud2 ostane
    cloud12 = cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original
    cloud12.Transform(transformMatrix12);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 w Transformation", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));
    cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    open3d::io::WritePointCloudToPLY("dddd10.ply", *cloudPtr1, true);
    //    open3d::io::WritePointCloudToPLY("dddd11.ply", *cloudPtr2, true);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "prozor normale", 640, 480, 50, 50, true);

    //    open3d::io::ReadPointCloud("dddd10.ply", cloud1);
    //    open3d::io::ReadPointCloud("dddd11.ply", cloud12);

    Eigen::Matrix4d transformMatrix12;
    transformMatrix12.setZero(4,4);
    transformMatrix12(0,0) = 1;
    transformMatrix12(1,1) = 1;
    transformMatrix12(2,2) = 1;
    transformMatrix12(3,3) = 1;


    /***   ICP   ***/
    double threshold = 0.005;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=50;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud1, cloud12,
                                                                                                                 threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    //    cout << "Fitness: " << reg.fitness_ << endl << endl;
    //    cout << "rmse" << reg.inlier_rmse_ << endl;

    cloud1.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    /***   ICP   ***/
}
void bCloudTransform13(int,void*)
{
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.0232;
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -4.05125; front[2] = -4.33795;   // drugi parametar pomiče gore dolje, što veći minus do je više, i treći se nešto može povećavati
    zoom = 0.0212;   // i zoom mi približava i udaljava
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "NACRT", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);


    open3d::io::ReadPointCloud("cloud1_002.ply", cloud1);
    open3d::io::ReadPointCloud("cloud2_002.ply", cloud2);
//    open3d::io::ReadPointCloud("3D_cam1_final.ply", cloud1);
//    open3d::io::ReadPointCloud("3D_cam2_final.ply", cloud2);

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

//    cloud1.PaintUniformColor({1, 0.706, 0});
//    cloud2.PaintUniformColor({0, 0.651, 0.929});
//    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
//    open3d::visualization::DrawGeometries({cloudPtr1});



    //***** OVDJE KOPIRATI cloud12 = cloud2, pa dalje s cloud12 raditi tako da cloud2 ostane
    cloud12 = cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original
    cloud12.Transform(transformMatrix12);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 w Transformation", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    open3d::io::WritePointCloudToPLY("dddd10.ply", *cloudPtr1, true);
    //    open3d::io::WritePointCloudToPLY("dddd11.ply", *cloudPtr2, true);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "prozor normale", 640, 480, 50, 50, true);

    //    open3d::io::ReadPointCloud("dddd10.ply", cloud1);
    //    open3d::io::ReadPointCloud("dddd11.ply", cloud12);

    Eigen::Matrix4d transformMatrix12;
    transformMatrix12.setZero(4,4);
    transformMatrix12(0,0) = 1;
    transformMatrix12(1,1) = 1;
    transformMatrix12(2,2) = 1;
    transformMatrix12(3,3) = 1;


    /***   ICP   ***/
    double threshold = 0.005;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=50;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud1, cloud12,
                                                                                                                 threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);


    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    //    cout << "Fitness: " << reg.fitness_ << endl << endl;
    //    cout << "rmse" << reg.inlier_rmse_ << endl;

    cloud1.Transform(reg12.transformation_);
    open3d::io::ReadPointCloud("3D_cam1_final.ply", cloud1);
    open3d::io::ReadPointCloud("3D_cam2_final.ply", cloud2);
    cloud2.Transform(transformMatrix12);
    cloud1.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    /***   ICP   ***/

}
void bCloudTransform123(int,void*)
{
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "TLOCRT", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    //    open3d::io::ReadPointCloud("cloud1_002.ply", cloud1);
    //    open3d::io::ReadPointCloud("cloud2_002.ply", cloud2);
    //    open3d::io::ReadPointCloud("cloud3_002.ply", cloud3);
    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_3.ply", cloud1);
    open3d::io::ReadPointCloud("3D_cedevita_cam2_002_3.ply", cloud2);
    open3d::io::ReadPointCloud("3D_cedevita_cam3_002_3.ply", cloud3);

//    open3d::io::ReadPointCloud("cloud1_br_50.ply", cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    open3d::visualization::DrawGeometries({cloudPtr1}, "All three camera together", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::io::ReadPointCloud("cloud2_br_50.ply", cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
//    open3d::visualization::DrawGeometries({cloudPtr2}, "All three camera together", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::io::ReadPointCloud("cloud3_br_50.ply", cloud3);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
//    open3d::visualization::DrawGeometries({cloudPtr3}, "All three camera together", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All 3 clouds", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);


//    cloud1.PaintUniformColor({1, 0.706, 0});
//    cloud3.PaintUniformColor({0, 0.651, 0.929});

    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3;
    point_cloud1= cloud1 + cloud2 + cloud3;

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
//    open3d::visualization::DrawGeometries({cloudPtr4}, "All three camera together", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    //    cloud1.PaintUniformColor({1, 0.706, 0});
    //    cloud2.PaintUniformColor({0, 0.651, 0.929});

    cloud12 = cloud2;
    cloud13 = cloud3;
    cloud12.Transform(transformMatrix12);
    cloud13.Transform(transformMatrix13);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds transformed", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);


    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud13.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_1.ply", *cloudPtr1, true);
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_2.ply", *cloudPtr2, true);
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_3.ply", *cloudPtr3, true);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds with normals", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    //    open3d::io::ReadPointCloud("cloud_nr1_1.ply", cloud1);
    //    open3d::io::ReadPointCloud("cloud_nr1_2.ply", cloud2);
    //    open3d::io::ReadPointCloud("cloud_nr1_3.ply", cloud3);


    //    cloud12 = point_cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original    TU NEŠTO NE RADI AHA NEMA TRANSFORMATION MATRIX. PRVI FIND STEREOCALIB ONDA OVO SVE
    //    cloud12.Transform(transformMatrix12);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;


    /***   ICP   ***/
    double threshold = 0.004;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
     * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=100;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud12, cloud1,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    open3d::pipelines::registration::RegistrationResult reg13 = open3d::pipelines::registration::RegistrationICP(cloud13, cloud1,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);


    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    cout << "ICP transformacijska matrica: " << endl << reg13.transformation_ << endl << endl;
    cout << "Fitness 12: " << reg12.fitness_ << endl;
    cout << "rmse12: " << reg12.inlier_rmse_ << endl << endl;
    cout << "Fitness 13: " << reg13.fitness_ << endl;
    cout << "rmse13: " << reg13.inlier_rmse_ << endl << endl;


    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_3.ply", cloud1);
    open3d::io::ReadPointCloud("3D_cedevita_cam2_002_3.ply", cloud12);
    open3d::io::ReadPointCloud("3D_cedevita_cam3_002_3.ply", cloud13);
    cloud12.Transform(transformMatrix12);
    cloud13.Transform(transformMatrix13);
    cloud12.Transform(reg12.transformation_);
    cloud13.Transform(reg13.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    point_cloud3 = cloud1;
    point_cloud3 = point_cloud3 + cloud12;
    point_cloud3 = point_cloud3 + cloud13;
    cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
//    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds after ICP", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr4}, "All clouds after ICP", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::io::WritePointCloudToPLY("proba1.ply", *cloudPtr4, true);







    open3d::io::ReadPointCloud("proba1.ply", cloud1);
    open3d::io::ReadPointCloud("proba2.ply", cloud2);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "iberlauf 1", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                   threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    cout << "Fitness 12: " << reg12.fitness_ << endl;
    cout << "rmse12: " << reg12.inlier_rmse_ << endl << endl;
    cloud2.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "iberlauf 2", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

}
/*** Calibration ***/

void bLiveStereo12(int, void*)
{
    int idNr = 0;
    while(idNr<12)
    {
        input1.release();
        input1 = fRealsenseImageGrabber1();// get<1> is mat opencv
        input2.release();
        input2 = fRealsenseImageGrabber2();// get<1> is mat opencv

        imshow("Output1", input1);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        imshow("Output2", input2);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        waitKey(1);
    }
}
void bLiveStereo13(int, void*)
{
    int idNr = 0;
    while(idNr<12)
    {
        input1.release();
        input1 = fRealsenseImageGrabber1();// get<1> is mat opencv
        input3.release();
        input3 = fRealsenseImageGrabber3();// get<1> is mat opencv

        imshow("Output1", input1);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        imshow("Output3", input3);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        waitKey(1);
    }
}
void bLiveStereo123(int, void*)
{
    int idNr = 0;
    while(idNr<12)
    {
        input1.release();
        input1 = fRealsenseImageGrabber1();// get<1> is mat opencv
        input2.release();
        input2 = fRealsenseImageGrabber2();// get<1> is mat opencv
        input3.release();
        input3 = fRealsenseImageGrabber3();// get<1> is mat opencv

        imshow("Output1", input1);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        imshow("Output2", input2);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        imshow("Output3", input3);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        waitKey(1);
    }
}
void bStereoSavePic(int, void*)
{
    Mat img1, img2;
    Mat gray1, gray2;
    Mat imgs1, imgs2;
    vector<Point2f> corners1, corners2;
    vector<vector<Point2f>> imagePoints[2];
    vector<vector<Point3f>> objectPoints;
    vector<Mat> rvecs1, rvecs2;
    vector<Mat> tvecs1, tvecs2;

    Mat intrinsic1 = Mat(3, 3, CV_32FC1);
    Mat intrinsic2 = Mat(3, 3, CV_32FC1);
    Mat distCoeffs1;
    Mat distCoeffs2;

    intrinsic1.ptr<float>(0)[0] = 1;
    intrinsic1.ptr<float>(1)[1] = 1;
    intrinsic2.ptr<float>(0)[0] = 1;
    intrinsic2.ptr<float>(1)[1] = 1;

    if( nrCam == 0)
    {
        cout << "Koju kameru(uz prvu) želite kalibrirati? [2 ili 3]: " << endl;
        cin >> nrCam;
    }

    input1.release();
    input1 = fRealsenseImageGrabber1();
    img1 = input1.clone();
    imgs1 = input1.clone();
    imshow("Output1", img1);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    if( nrCam == 2){
        input2.release();
        input2 = fRealsenseImageGrabber2();
        img2 = input2.clone();
        imgs2 = input2.clone();
        imshow("Output2", img2);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    }
    if( nrCam == 3){
        input3.release();
        input3 = fRealsenseImageGrabber3();
        img2 = input3.clone();
        imgs2 = input3.clone();
        imshow("Output3", img2);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    }

    waitKey(1);

    cvtColor(img1, gray1, COLOR_BGR2GRAY);
    cvtColor(img2, gray2, COLOR_BGR2GRAY);
    bool found1 = false;
    bool found2 = false;

    found1 = findChessboardCorners(img1, patternSize, corners1, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
    found2 = findChessboardCorners(img2, patternSize, corners2, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

    if (found1 && found2)
    {
        cornerSubPix(gray1, corners1, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cornerSubPix(gray2, corners2, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        vector< Point3f > obj;
        for (int i = 0; i < boardHeight; i++)
        {
            for (int j = 0; j < boardWidth; j++)
            {
                obj.push_back(Point3f((float)j * squareSize, (float)i * squareSize, 0));
            }
        }

        fDrawAxisAndCorners(img1,corners1,obj,"Output1",found1);

        if(nrCam == 2)
        {
            fDrawAxisAndCorners(img2,corners2,obj,"Output2",found2);
        }
        if(nrCam == 3)
        {
            fDrawAxisAndCorners(img2,corners2,obj,"Output3",found2);
        }


        vector<vector<Point2f>> cors[2];
        vector<vector<Point3f>> objs;
        cors[0].push_back(corners1);
        cors[1].push_back(corners2);
        objs.push_back(obj);

        double rms1 = calibrateCamera(objs, cors[0], img1.size(), intrinsic1, distCoeffs1, rvecs1, tvecs1, CALIB_FIX_K4|CALIB_FIX_K5);
        cout << "kontrolni rms kalibracije Cam 1 iznosi: " << rms1 << endl;

        double rms2 = calibrateCamera(objs, cors[1], img2.size(), intrinsic2, distCoeffs2, rvecs2, tvecs2, CALIB_FIX_K4|CALIB_FIX_K5);
        cout << "kontrolni rms kalibracije Cam " << nrCam << " iznosi: " << rms2 << endl;


        if(rms1 < 0.25 && rms2 < 0.25)
        {
            int x;
            cout << "Odgovara li orijentacija osi na slikama? Upišite: "<< endl << " 0 - Ne odgovara." << endl << " 1 - Odgovara." << endl;
            cin >> x;

            if(x==1)
            {
                nrPic = nrPic + 1;
                string idStr1;
                string idStr2;

                if(nrCam == 2)
                {
                    idStr1 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-2/cam1_img" + to_string(nrPic);
                    idStr2 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-2/cam2_img" + to_string(nrPic);
                }
                if(nrCam == 3)
                {
                    idStr1 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-3/cam1_img" + to_string(nrPic);
                    idStr2 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-3/cam3_img" + to_string(nrPic);
                }

                cout << idStr1 << endl << idStr2 << endl;
                imwrite(idStr1 + ".png", imgs1);
                imwrite(idStr2 + ".png", imgs2);
                cout << nrPic << ". par slika je spremljen." << endl << endl;

                if(nrPic == 9)
                {
                    nrCam = 0;
                    nrPic = 0;
                }
            }
            else
            {
                cout << "Os je krivo postavljena!" << endl;
            }
        }

        else
        {
            cout << "RMS slike nije zadovoljavajući!" << endl;
        }
    }
    waitKey(1);
}
void bFindStereoCalibration(int,void*)
{

    Mat img1, img2;
    Mat gray1, gray2;
    vector<Point2f> corners1, corners2;
    vector<vector<Point2f>> imagePoints[2];
    vector<vector<Point3f>> objectPoints;

    int idNr = 0;

    if( nrCam == 0)
    {
        cout << "Koju kameru(uz prvu) želite kalibrirati? [2 ili 3]: " << endl;
        cin >> nrCam;
    }

    for (int i = 0; i < 9; i++)        //ovaj cijela funkcija dalje bi trebala biti u foru dalje DO IMAGE POINTS
    {
        idNr = idNr + 1;
        string idStr1;
        string idStr2;

        if(nrCam == 2)
        {
            idStr1 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-2/cam1_img" + to_string(idNr) + ".png";    //MORAM FULL PATH: /home/ubuntu-z2/RobustVision_New/slika.png
            idStr2 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-2/cam2_img" + to_string(idNr) + ".png";
        }
        if(nrCam == 3)
        {
            idStr1 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-3/cam1_img" + to_string(idNr) + ".png";
            idStr2 = "/home/ubuntu-z2/RobustVision_New/Stereo_Images_1-3/cam3_img" + to_string(idNr) + ".png";
        }

        img1.empty();
        img2.empty();
        img1 = imread(idStr1);
        img2 = imread(idStr2);
        cvtColor(img1, gray1, COLOR_BGR2GRAY);
        cvtColor(img2, gray2, COLOR_BGR2GRAY);

        bool found1 = false, found2 = false;

        found1 = findChessboardCorners(img1, patternSize, corners1,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, patternSize, corners2,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);   //CALIB_CB_NORMALIZE_IMAGE

        if (found1 && found2)
        {
            //            if (!found1)
            //            {
            //                cout << "Cannot find chessboard corners." << endl;
            //                return make_tuple(Mat::zeros(Size(3,3), CV_64FC1),Mat::zeros(Size(3,1),CV_64FC1));
            //            }

            cornerSubPix(gray1, corners1, Size(5, 5), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray1, patternSize, corners1, found1);
            //            imshow(idStr1, gray1);


            cornerSubPix(gray2, corners2, Size(5, 5), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, patternSize, corners2, found2);
            //            imshow(idStr2, gray2);

            vector< Point3f > obj;
            for (int i = 0; i < boardHeight; i++)
            {
                for (int j = 0; j < boardWidth; j++)
                {
                    obj.push_back(Point3f((float)j * squareSize, (float)i * squareSize, 0));
                }
            }
            imagePoints[0].push_back(corners1);
            imagePoints[1].push_back(corners2);
            objectPoints.push_back(obj);
        }
        else
        {
            cout << "Chessboard corners not found!" << endl << endl;
        }
    }

    Mat intrinsic1, intrinsic2, distortionCoeff1, distortionCoeff2;
    double rms1, rms2;

    cv::FileStorage storage("/home/ubuntu-z2/RobustVision_New/Camera_1_Parameters.yml", cv::FileStorage::READ);
    storage["intrinsic1"] >> intrinsic1;
    storage["distCoeffs1"] >> distortionCoeff1;
    storage["rms1"] >> rms1;
    storage.release();

    if(nrCam == 2)
    {
        cv::FileStorage storage1("/home/ubuntu-z2/RobustVision_New/Camera_2_Parameters.yml", cv::FileStorage::READ);
        storage1["intrinsic2"] >> intrinsic2;
        storage1["distCoeffs2"] >> distortionCoeff2;
        storage1["rms2"] >> rms2;
        storage1.release();
    }
    if(nrCam == 3)
    {
        cv::FileStorage storage1("/home/ubuntu-z2/RobustVision_New/Camera_3_Parameters.yml", cv::FileStorage::READ);
        storage1["intrinsic3"] >> intrinsic2;
        storage1["distCoeffs3"] >> distortionCoeff2;
        storage1["rms3"] >> rms2;
        storage1.release();
    }

    //     if(intrinsic1.empty() || intrinsic2.empty())
    //     {
    //         nrCam = 1;
    //         auto camParameters = fSingleCamCalibration();
    //         intrinsic1 = std::get<0>(camParameters);   //   probati auto
    //         distortionCoeff1 = std::get<1>(camParameters);
    //         rms1 = std::get<2>(camParameters);

    //         nrCam = 2;
    //         camParameters = fSingleCamCalibration();
    //         intrinsic2 = std::get<0>(camParameters);        //    Mat intrinsic2 = std::get<0>(camParameters);
    //         distortionCoeff2 = std::get<1>(camParameters);      //    Mat distortionCoeff2 = std::get<1>(camParameters);
    //         rms2 = std::get<2>(camParameters);
    //     }

    cout << "Podaci 1. kamere: " << endl << "Intrinsic: " << intrinsic1 << endl << "Distorzija: " << distortionCoeff1 << endl << "RMS: " << rms1 << endl << endl;
    cout << "Podaci " << nrCam << ".kamere: " << endl << "Intrinsic: " << intrinsic2 << endl << "Distorzija: " << distortionCoeff2 << endl << "RMS: " << rms2 << endl << endl;

    // double rmsStereo = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1], intrinsic1, distortionCoeff1, intrinsic2, distortionCoeff2, img1.size(),
    //         R, T, E, F, CALIB_FIX_INTRINSIC | CALIB_FIX_FOCAL_LENGTH | CALIB_FIX_PRINCIPAL_POINT);    //CALIB_USE_INTRINSIC_GUESS  -  minimalna razlika,, CALIB_FIX_INTRINSIC koristiti sa cameraMatrix1

    double rmsStereo = stereoCalibrate(objectPoints, imagePoints[1], imagePoints[0], intrinsic2, distortionCoeff2, intrinsic1, distortionCoeff1, img1.size(),
            R, T, E, F, CALIB_FIX_INTRINSIC | CALIB_FIX_FOCAL_LENGTH | CALIB_FIX_PRINCIPAL_POINT);    //CALIB_USE_INTRINSIC_GUESS  -  minimalna razlika,, CALIB_FIX_INTRINSIC koristiti sa cameraMatrix1

    //    cameraMatrix1<<endl;
    //            cout<<"Distortion Coefficientsx1:"<<distortionCoefficients1

    cout << "RMS stereo kalibracije iznosi: " << rmsStereo << endl << endl;

    cout << "Matrica R" << R << endl;
    cout << "Matrica T" << T << endl << endl;

    if(nrCam == 2)
    {
        transformMatrix12.setZero(4,4);
        for (int i = 0; i < 3; i++)          //Rotaciju unesti u transform_2
        {
            int br = 0; Vec3d kp; kp.zeros(); kp=R.row(i);
            for (int j = 0; j < 3; j++)
            {
                transformMatrix12(i,j) = kp[br];
                br = br + 1;
            }
        }

        transformMatrix12(0,3) = T[0]/1000;
        transformMatrix12(1,3) = T[1]/1000;
        transformMatrix12(2,3) = T[2]/1000;
        transformMatrix12(3,3) = 1;
        cout << "Matrica transformacije Cloudova 1-2: " << endl << transformMatrix12 << endl << endl << endl << endl;
    }

    if(nrCam == 3)
    {
        transformMatrix13.setZero(4,4);
        for (int i = 0; i < 3; i++)          //Rotaciju unesti u transform_2
        {
            int br = 0; Vec3d kp; kp.zeros(); kp=R.row(i);
            for (int j = 0; j < 3; j++)
            {
                transformMatrix13(i,j) = kp[br];
                br = br + 1;
            }
        }

        transformMatrix13(0,3) = T[0]/1000;
        transformMatrix13(1,3) = T[1]/1000;
        transformMatrix13(2,3) = T[2]/1000;
        transformMatrix13(3,3) = 1;
        cout << "Matrica transformacije Cloudova 1-3: " << endl << transformMatrix13 << endl << endl << endl << endl;
    }

    nrCam = 0;

    waitKey(1);
}
//void bPomakZ(int, void*)
//{
//    int x,y,z;
//    cout << "Unesi pomak po osima: "<< endl << "X: "<< endl;
//    cin >> x;
//    cout << "Y: "<< endl;
//    cin >> y;
//    cout << "Z: "<< endl;
//    cin >> z;
//    transform_2.translation() << (T[0]-x)/1000, (T[1]-y)/1000, (T[2]-z)/1000;   //Translaciju unesti u transform_2
//}
void bSingleCamSave(int, void*)
{
    if( nrCam == 0)
    {
        cout << "Koju kameru želite kalibrirati? [1-4]: " << endl;
        cin >> nrCam;

        cout << "Ukoliko imate slike te želite samo kalibraciju unesite broj 9, inače unesite 0: " << endl;
        cin >> nrPic;
    }

    string izlaz = "Output" + to_string(nrCam);

    Mat img,imgs,gray;
    vector<Point2f> corners;
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    if(nrPic<9)
    {
        img.release();
        if(nrCam == 1)
        {
            img = fRealsenseImageGrabber1();
        }
        if(nrCam == 2)
        {
            img = fRealsenseImageGrabber2();
        }
        if(nrCam == 3)
        {
            img = fRealsenseImageGrabber3();
        }
        imgs = img.clone();
        imshow(izlaz, img);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        waitKey(1);

        cvtColor(img, gray, COLOR_BGR2GRAY);
        bool found = false;

        found = findChessboardCorners(img, patternSize, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (!found)
        {
            cout << "Cannot find chessboard corners." << endl;
        }

        else
        {
            cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            vector< Point3f > obj;
            vector< Point2f > objectPointsPlanar;

            for (int i = 0; i < boardHeight; i++)
            {
                for (int j = 0; j < boardWidth; j++)
                {
                    obj.push_back(Point3f((float)j * squareSize, (float)i * squareSize, 0));
                }
            }

            fDrawAxisAndCorners(img,corners,obj,izlaz,found);   //   Nacrtaj axis i cornerse na slici

            vector<vector<Point2f>> cors;
            vector<vector<Point3f>> objs;
            cors.push_back(corners);
            objs.push_back(obj);

            vector<Mat> rvecsCam, tvecsCam;
            Mat intrinsicCam = Mat(3, 3, CV_32FC1);
            Mat distCoeffsCam;
            intrinsicCam.ptr<float>(0)[0] = 1;
            intrinsicCam.ptr<float>(1)[1] = 1;

            double rmsCam = calibrateCamera(objs, cors, img.size(), intrinsicCam, distCoeffsCam, rvecsCam, tvecsCam, CALIB_FIX_K4|CALIB_FIX_K5);
            cout << "kontrolni rms kalibracije iznosi: " << rmsCam << endl << endl;

            if(rmsCam < 0.15)
            {
                int x;
                cout << "Odgovara li orijentacija osi na slici? Upišite: "<< endl << " 0 - slika ne odgovara." << endl << " 1 - slika odgovara." << endl;
                cin >> x;

                if(x==1)
                {
                    nrPic = nrPic + 1;
                    string idStr = "images_saved_cam" + to_string(nrCam) + "/pic" + to_string(nrPic);
                    cout << idStr << endl;

                    imwrite(idStr + ".png", imgs);

                    cout << nrPic << ".  slika je spremljena." << endl << endl;
                }
                else
                {
                    cout << "Os je krivo postavljena na slici!" << endl;
                }
            }

            else
            {
                cout << "RMS slike nije zadovoljavajući!" << endl;
            }
        }
    }


    if(nrPic == 9)
    {
        if( nrCam == 1)
        {
            auto camParameters = fSingleCamCalibration();
            Mat intrinsic = std::get<0>(camParameters);
            Mat distCoeffs = std::get<1>(camParameters);
        }
        if( nrCam == 2)
        {
            auto camParameters = fSingleCamCalibration();
            Mat intrinsic = std::get<0>(camParameters);
            Mat distCoeffs = std::get<1>(camParameters);
        }
        if( nrCam == 3)
        {
            auto camParameters = fSingleCamCalibration();
            Mat intrinsic = std::get<0>(camParameters);
            Mat distCoeffs = std::get<1>(camParameters);
        }

        nrPic = 0;
        nrCam = 0;
    }
}
void bSingleCamLive(int, void*)
{
    if(nrCam == 0)
    {
        cout << "Koju kameru želite kalibrirati? [1-4]: "<< endl;
        cin >> nrCam;
    }

    Mat img;
    string izlaz = "Output" + to_string(nrCam);

    while(6 != 10)
    {
        img.release();

        if(nrCam == 1)
        {
            img = fRealsenseImageGrabber1();// get<1> is mat opencv
        }
        if(nrCam == 2)
        {
            img = fRealsenseImageGrabber2();// get<1> is mat opencv
        }
        if(nrCam == 3)
        {
            img = fRealsenseImageGrabber3();// get<1> is mat opencv
        }

        imshow(izlaz, img);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
        waitKey(1);
    }
}
//void bOpen3dVisualizer(int, void*)
//{
//    auto point_cloud = open3d::geometry::PointCloud();
//    open3d::io::ReadPointCloud("fragment1.ply", point_cloud);
//    auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>(point_cloud);
//    vis.AddGeometry(cloud_ptr);
//    vis.UpdateRender();
//    vis.Run();
// Create Visualizer and point cloud
//    open3d::visualization::Visualizer vis;
//    vis.CreateVisualizerWindow("Open3D");
//    std::vector<Eigen::Vector3d> pointssss = std::vector<Eigen::Vector3d>();
//    for (int i = 0; i < 1000; i++)
//    {
//        auto tmp = Eigen::Vector3d(rand() % 100, rand() % 100, rand() % 100);
//        pointssss.push_back(tmp);
//    }
//    auto tmpPointCloud1 = open3d::geometry::PointCloud();
//   tmpPointCloud1.points_ = pointssss;
//    tmpPointCloud1.colors_ = pointssss1;
//    tmpPointCloud2.PaintUniformColor({1, 0.706, 0});
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(tmpPointCloud1);
//    open3d::io::WritePointCloudToPLY("dddd1.ply", *cloud_ptr, true);
//    vis.AddGeometry(cloudPtr1);
//    vis.UpdateRender();
//    vis.Run();
//}
void bCloudCleaning(int, void*)
{
    //PRIMJER SA OPEN3D STRANICA
    //    open3d::geometry::PointCloud point_cloud1, point_cloud2;
    //    open3d::io::ReadPointCloud("fragment_vox.ply", point_cloud1);
    //    open3d::io::ReadPointCloud("fragment_vox.ply", point_cloud2);
    ////    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = point_cloud1.VoxelDownSample(0.025);
    ////    open3d::io::WritePointCloudToPLY("fragment_vox.ply", *cloudPtr1, true);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    Eigen::Vector3d translacija;
    //    translacija(0)=0.05;
    //    translacija(1)=0.05;
    //    translacija(2)=0.1;
    //    cout << "Matrica transformacije Cloudova 1-2: " << endl << transformMatrix12 << endl;
    //    cloud12 = point_cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original    TU NEŠTO NE RADI AHA NEMA TRANSFORMATION MATRIX. PRVI FIND STEREOCALIB ONDA OVO SVE
    //    cloud12.Translate(translacija);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});
    //    transformMatrix12.setZero(4,4);
    //    transformMatrix12(0,0) = 1;
    //    transformMatrix12(1,1) = 1;
    //    transformMatrix12(2,2) = 1;
    //    transformMatrix12(3,3) = 1;
    // SAMO JOŠ ICP



    //                        AKO ŽELIM UZETI NOVE CLOUDOVE
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::io::WritePointCloudToPLY("dddd1.ply", *cloudPtr1, true);
    //    open3d::visualization::DrawGeometries({cloudPtr1});
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    //    open3d::io::WritePointCloudToPLY("dddd2.ply", *cloudPtr2, true);
    //    open3d::visualization::DrawGeometries({cloudPtr2});
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    open3d::geometry::PointCloud point_cloud1, point_cloud2;   // može i ovako: auto point_cloud1 = open3d::geometry::PointCloud();

    open3d::io::ReadPointCloud("dddd3.ply", point_cloud1);
    open3d::io::ReadPointCloud("dddd4.ply", point_cloud2);

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    //    point_cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.02 * 2.0, 30));
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr1});
    //    open3d::io::WritePointCloudToPLY("dddd7.ply", *source_down, true);

    //    cloudPtr1 = point_cloud1.VoxelDownSample(0.0002);
    //    open3d::io::WritePointCloudToPLY("dddd3.ply", *cloudPtr1, true);
    //    cloudPtr2 = point_cloud2.VoxelDownSample(0.0002);
    //    open3d::io::WritePointCloudToPLY("dddd4.ply", *cloudPtr2, true);
    //    open3d::io::ReadPointCloud("dddd3.ply", point_cloud1);
    //    open3d::io::ReadPointCloud("dddd4.ply", point_cloud2);
    point_cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));
    point_cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    open3d::io::WritePointCloudToPLY("dddd5.ply", *cloudPtr1, true);
    open3d::io::WritePointCloudToPLY("dddd6.ply", *cloudPtr2, true);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "prozor", 640, 480, 50, 50, true);


    open3d::io::ReadPointCloud("dddd5.ply", point_cloud1);
    open3d::io::ReadPointCloud("dddd6.ply", point_cloud2);


    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    /******************************* REMOVE STATICAL OUTLIERS ******************************/
    //    open3d::geometry::PointCloud point_cloud3;
    ////    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
    //    open3d::io::ReadPointCloud("dddd3.ply", point_cloud3);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
    //    open3d::visualization::DrawGeometries({cloudPtr3});

    //    tuple <std::shared_ptr<open3d::geometry::PointCloud>, std::vector<long unsigned int>> geek;
    //    geek = point_cloud3.RemoveStatisticalOutliers(50, 5.0, true);
    //    cloudPtr3 = std::get<0>(geek);
    //    std::vector<long unsigned int> Ind = std::get<1>(geek);
    ////    open3d::visualization::DrawGeometries({cloudPtr3});
    //    open3d::io::WritePointCloudToPLY("dddd7.ply", *cloudPtr3, true);

    //    open3d::io::ReadPointCloud("dddd7.ply", point_cloud3);
    //    point_cloud3.PaintUniformColor({1, 0.706, 0});
    //    cloudPtr3 = point_cloud3.SelectByIndex(Ind, true);
    //    point_cloud3.PaintUniformColor({0, 0.651, 0.929});
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = point_cloud3.SelectByIndex(Ind, false);
    //    open3d::visualization::DrawGeometries({cloudPtr3, cloudPtr4});


    //***************************************************** Axis aigned bounding box
    //    auto clcl = point_cloud3.GetAxisAlignedBoundingBox();
    //    clcl.color_({1,0,0});
    //    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> cloudPtr8 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(clcl);
    //    open3d::visualization::DrawGeometries({cloudPtr8});
    // **************************************************Cluster DBSCAN
    //    std::vector<int> labels = point_cloud3.ClusterDBSCAN(0.02, 10, true);
    //    point_cloud3.SegmentPlane();


    //***** OVDJE KOPIRATI cloud12 = cloud2, pa dalje s cloud12 raditi tako da cloud2 ostane
    cloud12 = point_cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original    TU NEŠTO NE RADI AHA NEMA TRANSFORMATION MATRIX. PRVI FIND STEREOCALIB ONDA OVO SVE
    cloud12.Transform(transformMatrix12);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    //    open3d::io::ReadPointCloud("dddd1.ply", cloud1);
    //    open3d::io::ReadPointCloud("dddd2.ply", cloud2);
    //    cloud2.Transform(transformMatrix12);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);


    Eigen::Matrix4d transformMatrix12;
    transformMatrix12.setZero(4,4);
    transformMatrix12(0,0) = 1;
    transformMatrix12(1,1) = 1;
    transformMatrix12(2,2) = 1;
    transformMatrix12(3,3) = 1;


    //    open3d::pipelines::registration::RegistrationResult resultt = open3d::pipelines::registration::EvaluateRegistration(point_cloud1, cloud12, 0.05, transformMatrix12);
    //    cout << resultt.transformation_ << endl << endl;
    //    cout << resultt.fitness_ << endl << endl;
    //    cout << resultt.inlier_rmse_ << endl;


    /***   ICP   ***/
    double threshold = 0.005;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
     * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=1000;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    //    open3d::pipelines::registration::RegistrationResult reg = open3d::pipelines::registration::RegistrationICP(point_cloud1, cloud12,
    //                                threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationPointToPoint(false), crit);

    //    open3d::pipelines::registration::RegistrationResult reg = open3d::pipelines::registration::RegistrationColoredICP(point_cloud1, cloud12,
    //                                threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationForColoredICP(), crit);

    //    open3d::pipelines::registration::RegistrationResult reg = open3d::pipelines::registration::RegistrationGeneralizedICP(point_cloud1, cloud12,
    //                                                            threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationForGeneralizedICP(), crit);


    open3d::pipelines::registration::RegistrationResult reg = open3d::pipelines::registration::RegistrationICP(point_cloud1, cloud12,
                                                                                                               threshold, transformMatrix12, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);


    cout << "ICP transformacijska matrica: " << endl << reg.transformation_ << endl << endl;
    cout << "Fitness: " << reg.fitness_ << endl << endl;
    cout << "rmse" << reg.inlier_rmse_ << endl;

    point_cloud1.Transform(reg.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});


    //    cloud1.Transform(reg.transformation_);
    //    cloud1.PaintUniformColor({1, 0.706, 0});
    //    cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});
    /***   ICP   ***/


    //    open3d::visualization::Visualizer vis1;
    //    vis1.CreateVisualizerWindow("Visualizer 1", 960, 540, 0, 0);
    //    vis1.AddGeometry(cloudPtr1);
    //    vis1.AddGeometry(cloudPtr2);
    ////    vis1.UpdateRender();
    ////    vis1.Run();

    //    double threshold = 0.05;
    //    cout << "Kreće ICP:" << endl;
    //    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    //    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
    //     * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    //    crit.max_iteration_=1;
    //    crit.relative_fitness_=1.000000;
    //    crit.relative_rmse_=1.000000e-06;

    //    for(int i=0; i<100; i++)
    //    {
    //        open3d::pipelines::registration::RegistrationResult reg2 = open3d::pipelines::registration::RegistrationICP(point_cloud1, cloud12,
    //                                    threshold, transformMatrix15, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
    //        cout << "ICP transformacijska matrica: " << endl << reg2.transformation_ << endl << endl;
    //        point_cloud1.Transform(transformMatrix12);
    //        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //        vis1.UpdateGeometry();
    //        vis1.PollEvents();
    //        vis1.UpdateRender();
    //    }
    //    cout << "ENDE DES" << endl;
    //    vis1.DestroyVisualizerWindow();

}
void bsavepics(int, void*)
{

    Mat img1;
    int i=1, j=1;
    string ObjectNr = "P";

    std::chrono::time_point<std::chrono::steady_clock> previous_time = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = previous_time-current_time;

    ObjectNr = ObjectNr + to_string(j);

    while (elapsed_seconds.count() < 5)
    {
        if (std::experimental::filesystem::exists(ObjectNr) == false)
        {
            std::experimental::filesystem::create_directory(ObjectNr);
        }

        if (std::experimental::filesystem::exists(ObjectNr + "/2D_" + ObjectNr) == false)
        {
            std::experimental::filesystem::create_directory(ObjectNr + "/2D_" + ObjectNr);
        }

        if (std::experimental::filesystem::exists(ObjectNr + "/2D_" + ObjectNr + "/C1_2D_" + ObjectNr) == false)
        {
            std::experimental::filesystem::create_directory(ObjectNr + "/2D_" + ObjectNr + "/C1_2D_" + ObjectNr);
        }

        img1.release();
        //        input1 = fRealsenseImageGrabber1();
        rs2::frameset frames = pipeReal1.wait_for_frames();
        rs2::video_frame colorFrame=frames.get_color_frame();
        cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
        img1 = colorImage.clone();
        //        img1 = input1.clone();

        string idStr = ObjectNr + "/2D_" + ObjectNr + "/C1_2D_" + ObjectNr + "/Cam1_P" + to_string(j) + "_pic" + to_string(i);
        cout << idStr << endl;
        imwrite(idStr + ".png", img1);

        pokusaj1.push_back(frames.get_depth_frame());
        pokusaj2.push_back(frames.get_color_frame());

        current_time = std::chrono::steady_clock::now();
        elapsed_seconds = current_time-previous_time;

        i = i + 1;
    }


    rs2::video_frame colorFrame=pokusaj2[50];
    Mat colorImage1(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    img1 = colorImage1.clone();
    imwrite("druga.png", img1);

    colorFrame=pokusaj2[140];
    Mat colorImage3(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    img1 = colorImage3.clone();
    imwrite("cetvrta.png", img1);
}

void bsaveframes(int, void*)
{

    //    pipeReal1.start(cfg1);  //****** SAVE PICTURES AND CLOUDS ******
    std::chrono::time_point<std::chrono::steady_clock> previous_time = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = previous_time-current_time;
    std::chrono::time_point<std::chrono::steady_clock> a = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> b = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> c = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> d = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_time = a-b;
    open3d::geometry::AxisAlignedBoundingBox boxxx1, boxxx2, boxxx3;
    boxxx1.min_bound_[0] = -0.17; boxxx1.min_bound_[1] = -0.15; boxxx1.min_bound_[2] = 0.2;
    boxxx1.max_bound_[0] = 0.20; boxxx1.max_bound_[1] = 0.13; boxxx1.max_bound_[2] = 0.5;
    boxxx1.color_[0] = 1; boxxx1.color_[1] = 0; boxxx1.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu1 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx1);

    boxxx2.min_bound_[0] = -0.11; boxxx2.min_bound_[1] = -0.05; boxxx2.min_bound_[2] = 0.15;
    boxxx2.max_bound_[0] = 0.2; boxxx2.max_bound_[1] = 0.135; boxxx2.max_bound_[2] = 0.45;
    boxxx2.color_[0] = 1; boxxx2.color_[1] = 0; boxxx2.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu2 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx2);

    boxxx3.min_bound_[0] = -0.18; boxxx3.min_bound_[1] = -0.05; boxxx3.min_bound_[2] = 0.15;
    boxxx3.max_bound_[0] = 0.13; boxxx3.max_bound_[1] = 0.135; boxxx3.max_bound_[2] = 0.46;
    boxxx3.color_[0] = 1; boxxx3.color_[1] = 0; boxxx3.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu3 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx3);


    int i = 1;
    std::vector<rs2::frameset> framesS1, framesS2, framesS3;
    framesS1.reserve(500);
    framesS2.reserve(500);
    framesS3.reserve(500);
//******************* od ovdje zakomentirati
    //************************************ UZIMANJE FRAMEOVA *********************************
//    previous_time = std::chrono::steady_clock::now();
//    while (elapsed_seconds.count() < 4)
//    {

//        frames1 = pipeReal1.wait_for_frames();
//        frames2 = pipeReal2.wait_for_frames();
//        frames3 = pipeReal3.wait_for_frames();
//        frames1.keep();
//        frames2.keep();
//        frames3.keep();
//        framesS1.push_back(frames1);
//        framesS2.push_back(frames2);
//        framesS3.push_back(frames3);

//        cout << "Broj iteracija: " << i << endl;
//        i=i+1;
//        cout << "Vector size: " << framesS1.size() << endl;
//        cout << "Vector capacity: " << framesS1.capacity() << endl;

//        current_time = std::chrono::steady_clock::now();
//        elapsed_seconds = current_time-previous_time;
//        cout << elapsed_seconds.count() << endl << endl;
//    }
//    //************************************ UZIMANJE FRAMEOVA *********************************

//    rs2::threshold_filter thresholdFilter;
//    rs2::points points1, points2, points3;
//    string naziv1, naziv2, naziv3;


//    for(i = 1; i < framesS1.size(); i++)
//    {
//        frames1 = framesS1[i];
//        frames2 = framesS2[i];
//        frames3 = framesS3[i];

//        // 1.cam : R=110,255 ; G=30,255 ; B=12,36
//        // 2.cam : R=100,255 ; G=50,255 ; B=14,36
//        // 3.cam : R=110,255 ; G=30,255 ; B=15,36

//        Mat hsv_image, frame_threshold, img_bw, imgbw_flat;
//        std::vector <int> veki;
//        std::vector <long unsigned int> rez_vect1, rez_vect2, rez_vect3;
//        //************************************ UZIMANJE FRAMEOVA *********************************
//        //********************************************** Spremanje slika **********************************************
//        rs2::video_frame color1 = frames1.get_color_frame();
//        Mat colorImage1(cv::Size(color1.get_width(), color1.get_height()), CV_8UC3, (void*)color1.get_data(), cv::Mat::AUTO_STEP);   // t = 4.83e-04 s
//        cvtColor(colorImage1, hsv_image, COLOR_BGR2HSV);
//        inRange(hsv_image,Scalar(12,30,110), Scalar(36,255,255),frame_threshold);  // inRange(hsv_image,Scalar(15,30,110), Scalar(36,255,255),frame_threshold);
//        bitwise_not(frame_threshold, frame_threshold);
//        cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
//        cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
//        bitwise_and(colorImage1, frame_threshold, colorImage1);
//        imwrite("prva" + to_string(i) + ".png", colorImage1);   // t = 0.01s

//        imgbw_flat = img_bw.reshape(1,1); // flat 1d
//        veki = imgbw_flat.row(0);
//        for (int j = 0; j < veki.size(); j++) {
//            if(veki.at(j) == 255){
//                veki.at(j) = 1;
//                veki.at(j) = veki.at(j) * j;
//                rez_vect1.push_back(veki.at(j));
//            }
//        }


//        rs2::video_frame color2 = frames2.get_color_frame();
//        Mat colorImage2(cv::Size(color2.get_width(), color2.get_height()), CV_8UC3, (void*)color2.get_data(), cv::Mat::AUTO_STEP);   // t = 4.83e-04 s
//        cvtColor(colorImage2, hsv_image, COLOR_BGR2HSV);
//        inRange(hsv_image,Scalar(14,50,100), Scalar(36,255,255),frame_threshold);  // inRange(hsv_image,Scalar(12,30,110), Scalar(36,255,255),frame_threshold);
//        bitwise_not(frame_threshold, frame_threshold);
//        cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
//        cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
//        bitwise_and(colorImage2, frame_threshold, colorImage2);
//        imwrite("druga" + to_string(i) + ".png", colorImage2);   // t = 0.01s

//        imgbw_flat = img_bw.reshape(1,1); // flat 1d
//        veki = imgbw_flat.row(0);
//        for (int j = 0; j < veki.size(); j++) {
//            if(veki.at(j) == 255){
//                veki.at(j) = 1;
//                veki.at(j) = veki.at(j) * j;
//                rez_vect2.push_back(veki.at(j));
//            }
//        }


//        rs2::video_frame color3 = frames3.get_color_frame();
//        Mat colorImage3(cv::Size(color3.get_width(), color3.get_height()), CV_8UC3, (void*)color3.get_data(), cv::Mat::AUTO_STEP);   // t = 4.83e-04 s
//        cvtColor(colorImage3, hsv_image, COLOR_BGR2HSV);
//        inRange(hsv_image,Scalar(15,30,110), Scalar(36,255,255),frame_threshold);  // inRange(hsv_image,Scalar(12,30,110), Scalar(36,255,255),frame_threshold);
//        bitwise_not(frame_threshold, frame_threshold);
//        cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
//        cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
//        bitwise_and(colorImage3, frame_threshold, colorImage3);
//        imwrite("treca" + to_string(i) + ".png", colorImage3);   // t = 0.01s

//        imgbw_flat = img_bw.reshape(1,1); // flat 1d
//        veki = imgbw_flat.row(0);//    fInitializeDevices();
//        for (int j = 0; j< veki.size(); j++) {
//            if(veki.at(j) == 255){
//                veki.at(j) = 1;
//                veki.at(j) = veki.at(j) * j;
//                rez_vect3.push_back(veki.at(j));
//            }
//        }
//        //********************************************** Spremanje slika **********************************************

//        frames1 = align_to_depth1.process(frames1);
//        rs2::depth_frame depth1 = frames1.get_depth_frame();   //  t = 0.00348485 s
//        frames2 = align_to_depth1.process(frames2);
//        rs2::depth_frame depth2 = frames2.get_depth_frame();   //  t = 0.00348485 s
//        frames3 = align_to_depth1.process(frames3);
//        rs2::depth_frame depth3 = frames3.get_depth_frame();   //  t = 0.00348485 s

//        thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.25f);   //   daljina na kojoj da hvata point cloud
//        thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);   //  t = 0.000399798 s
//        depth1 = thresholdFilter.process(depth1);
//        depth2 = thresholdFilter.process(depth2);
//        depth3 = thresholdFilter.process(depth3);

//        const int w = color1.get_width();    //   da bi dobili dimenzije matrice?
//        const int h = color1.get_height();   //  t = 1.54e-07 s

//        pc1.map_to(color1);
//        points1 = pc1.calculate(depth1);   //   t = 0.002 s
//        pc2.map_to(color2);
//        points2 = pc2.calculate(depth2);   //   t = 0.002 s
//        pc3.map_to(color3);
//        points3 = pc3.calculate(depth3);   //   t = 0.002 s


//        cloud1.Clear();
//        cloud1 = open3d::geometry::PointCloud(fPointsToPc(points1, color1));   //  t = 0.0372791 s
//        cloud2.Clear();
//        cloud2 = open3d::geometry::PointCloud(fPointsToPc(points2, color2));   //  t = 0.0372791 s
//        cloud3.Clear();
//        cloud3 = open3d::geometry::PointCloud(fPointsToPc(points3, color3));   //  t = 0.0372791 s

//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1;   //   t = 0.00161014 s
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2;   //   t = 0.00161014 s
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3;   //   t = 0.00161014 s

//        cloudPtr1 = cloud1.SelectByIndex(rez_vect1);
//        cloudPtr2 = cloud1.SelectByIndex(rez_vect2);
//        cloudPtr3 = cloud1.SelectByIndex(rez_vect3);

//        naziv1 = "cloud1_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s
//        naziv2 = "cloud2_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s
//        naziv3 = "cloud3_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s

//        open3d::io::WritePointCloudToPLY(naziv1, *cloudPtr1, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//        open3d::io::WritePointCloudToPLY(naziv2, *cloudPtr2, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//        open3d::io::WritePointCloudToPLY(naziv3, *cloudPtr3, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample

//        open3d::io::ReadPointCloud(naziv1, cloud1);
//        open3d::io::ReadPointCloud(naziv2, cloud2);
//        open3d::io::ReadPointCloud(naziv3, cloud3);

//        cloudPtr1 = cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
//        cloudPtr2 = cloud2.VoxelDownSample(0.002);   //  t = 0.00780749 s
//        cloudPtr3 = cloud3.VoxelDownSample(0.002);   //  t = 0.00780749 s

//        cloudPtr1 = cloud1.Crop(boxxx1);
//        cloudPtr2 = cloud2.Crop(boxxx2);
//        cloudPtr3 = cloud3.Crop(boxxx3);
//        //    open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

////        naziv1 = "cloud1_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s
////        naziv2 = "cloud2_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s
////        naziv3 = "cloud3_br_" + to_string(i) + ".ply";   //   t = 4.455e-06 s

//        open3d::io::WritePointCloudToPLY(naziv1, *cloudPtr1, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//        open3d::io::WritePointCloudToPLY(naziv2, *cloudPtr2, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//        open3d::io::WritePointCloudToPLY(naziv3, *cloudPtr3, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample


//        current_time = std::chrono::steady_clock::now();
//        elapsed_seconds = current_time-previous_time;
//        cout << elapsed_seconds.count() << endl;    //   t = t = 4.455e-05 s

//    }
//****** SAVE PICTURES AND CLOUDS ****** do ovdje zakomentirati


//************************************************* SPREMANJE SLIKA ZA MACHINE LEARNING*************************************************
    //************************************ UZIMANJE FRAMEOVA *********************************
    previous_time = std::chrono::steady_clock::now();
    while (elapsed_seconds.count() < 6.5)
    {

        frames1 = pipeReal1.wait_for_frames();
        frames2 = pipeReal2.wait_for_frames();
        frames3 = pipeReal3.wait_for_frames();
        frames1.keep();
        frames2.keep();
        frames3.keep();
        framesS1.push_back(frames1);
        framesS2.push_back(frames2);
        framesS3.push_back(frames3);

        cout << "Broj iteracija: " << i << endl;
        i = i + 1;
        cout << "Vector size: " << framesS1.size() << endl;
        cout << "Vector capacity: " << framesS1.capacity() << endl;

        current_time = std::chrono::steady_clock::now();
        elapsed_seconds = current_time - previous_time;
        cout << elapsed_seconds.count() << endl
             << endl;
    }

    for (i = 1; i < framesS1.size(); i++) // od tuda zakomentirati
    {
        frames1 = framesS1[i];
        frames2 = framesS2[i];
        frames3 = framesS3[i];

        rs2::video_frame color1 = frames1.get_color_frame();
        Mat colorImage1(cv::Size(color1.get_width(), color1.get_height()), CV_8UC3, (void *)color1.get_data(), cv::Mat::AUTO_STEP); // t = 4.83e-04 s
        imwrite("/home/ubuntu-z2/RobustVision_New/Cedevita_pic_set/cede_cam1_img" + to_string(i) + ".jpg", colorImage1);              // t = 0.01s

        rs2::video_frame color2 = frames2.get_color_frame();
        Mat colorImage2(cv::Size(color2.get_width(), color2.get_height()), CV_8UC3, (void *)color2.get_data(), cv::Mat::AUTO_STEP); // t = 4.83e-04 s
        imwrite("/home/ubuntu-z2/RobustVision_New/Cedevita_pic_set/cede_cam2_img" + to_string(i) + ".jpg", colorImage2);              // t = 0.01s

        rs2::video_frame color3 = frames3.get_color_frame();
        Mat colorImage3(cv::Size(color3.get_width(), color3.get_height()), CV_8UC3, (void *)color3.get_data(), cv::Mat::AUTO_STEP); // t = 4.83e-04 s
        imwrite("/home/ubuntu-z2/RobustVision_New/Cedevita_pic_set/cede_cam3_img" + to_string(i) + ".jpg", colorImage3);              // t = 0.01s
        cout << "slike broj: " << i << endl;
    }

//************************************************* SPREMANJE SLIKA ZA MACHINE LEARNING*************************************************


//string naziv;
//open3d::geometry::PointCloud point_cloud1, point_cloud2;
//naziv = "cloud3_br_" + to_string(5) + ".ply";
//open3d::io::ReadPointCloud(naziv, point_cloud1);
//std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//open3d::visualization::DrawGeometries({cloudPtr1});

//naziv = "cloud3_br_" + to_string(60) + ".ply";
//open3d::io::ReadPointCloud(naziv, point_cloud2);
//std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
//open3d::visualization::DrawGeometries({cloudPtr2});

//    rs2::video_frame colorFrame=framesS1[1].get_color_frame();
//    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
//    Mat img1 = colorImage.clone();
//    imwrite("prva.png", img1);

//    colorFrame=framesS1[50].get_color_frame();
//    Mat colorImage1(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage1.clone();
//    imwrite("druga.png", img1);

//    cout << "Broj iteracija: " << i << endl;
//    pipeReal1.stop();
}

void bsavecloud(int, void*)
{
    //    pipeReal1.start(cfg1);
    open3d::geometry::PointCloud point_cloud1, point_cloud2;

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;

    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;
    //    std::array<Eigen::Matrix4d_u, 100> uk_transform;
    std::array<Eigen::Matrix4d_u, 100> transformacije;
    transformacije[0] = transformMat;
    int j = 1;
    //    open3d::io::ReadPointCloud("cloud1_br_12.ply", point_cloud1);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr3}, "point u koji ce se zbrajati", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);


    //    for(int i=8; i<60;i++){

    cloud1.Clear();
    cloud2.Clear();
    //    open3d::io::ReadPointCloud("cloud1_br_" + to_string(i) + ".ply", cloud1);
    //    open3d::io::ReadPointCloud("cloud1_br_" + to_string(i-7) + ".ply", cloud2);
    open3d::io::ReadPointCloud("cloud1_br_1.ply", cloud1);
    open3d::io::ReadPointCloud("cloud1_br_8.ply", cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));

    open3d::geometry::AxisAlignedBoundingBox boxxx;
    boxxx.min_bound_[0] = -0.3; boxxx.min_bound_[1] = -0.3; boxxx.min_bound_[2] = 0.2;
    boxxx.max_bound_[0] = 0.5; boxxx.max_bound_[1] = 0.15; boxxx.max_bound_[2] = 0.7;
    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx);
    //        open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloudPtr1 = cloud1.Crop(boxxx);
    cloudPtr2 = cloud2.Crop(boxxx);
    //    open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::io::WritePointCloudToPLY("rez_1.ply", *cloudPtr1, true);
    open3d::io::WritePointCloudToPLY("rez_2.ply", *cloudPtr2, true);
    open3d::io::ReadPointCloud("rez_1.ply", cloud1);
    open3d::io::ReadPointCloud("rez_2.ply", cloud2);

    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    //        open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);


    /***   ICP   ***/
    double threshold = 0.05;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=50;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    Eigen::Matrix4d_u mul;
    for (int f = 0; f < 4; f++) {
        for (int g = 0; g < 4; g++) {
            mul(f, g) = 0;

            for (int h = 0; h < 4; h++) {
                mul(f, g) += transformacije[j-1](f, h) * reg12.transformation_(h, g);
            }

            //                cout << mul(i, j) << "\t";
        }

        //            cout << endl;
    }
    cout << "mul je:" << mul << endl;

    transformacije[j] = mul;

    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    //    cout << "Fitness: " << reg.fitness_ << endl << endl;
    //    cout << "rmse" << reg.inlier_rmse_ << endl;

    cloud12 = cloud2;
    cloud12.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);


    cloud1.Transform(transformacije[j-1]);
    cloud2.Transform(mul);
    //        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);


    point_cloud1 += cloud1;
    point_cloud1 += cloud2;

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    open3d::visualization::DrawGeometries({cloudPtr3}, "BoX2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    //    cout << "i iznosi: " << i << endl;
    //    i = i + 6;
    j = j + 1;

    /***   ICP   ***/



    //    }

    //    pipeReal1.stop();
}
void breadcloud(int, void*)
{
    /*             IZDVAJANJE ŽUTOG IZ SLIKE
//    Mat img1;
//    img1.release();
//    //        input1 = fRealsenseImageGrabber1();
//    rs2::frameset frames = pipeReal1.wait_for_frames();
//    rs2::video_frame colorFrame=frames.get_color_frame();
//    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage.clone();
//    imwrite("zutobre.png", img1);
    Mat img, gray;
    img.empty();
    img = imread("zutobre.png");
    cvtColor(img, gray, COLOR_BGR2GRAY);
    imshow("Output1", img);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    Mat hsv_image;
    cvtColor(img, hsv_image, COLOR_BGR2HSV);
    imshow("Output2", hsv_image);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
//    Mat imgThreshold, pic, img5;
////    inRange(hsv_image, Scalar(15, 0, 0), Scalar(36, 255, 255), imgThreshold);  // separate yellow
//    inRange(img, Scalar(20, 93, 112), Scalar(31, 227, 247), imgThreshold);  // separate yellow
//    imwrite("probaHSV.png", imgThreshold);
//    imshow("Output3", img5);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    waitKey(1);   //čekaj tipku
*/

    // CLOUDOVIIIIIIIIIIIIIIII

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.0232;
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -4.05125; front[2] = -4.33795;   // drugi parametar pomiče gore dolje, što veći minus do je više, i treći se nešto može povećavati
    zoom = 0.0212;   // i zoom mi približava i udaljava
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "NACRT", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    open3d::io::ReadPointCloud("cloud1_br_17.ply", cloud1);
    open3d::io::ReadPointCloud("cloud1_br_23.ply", cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    //***** OVDJE KOPIRATI cloud12 = cloud2, pa dalje s cloud12 raditi tako da cloud2 ostane
    cloud12 = cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original
    //    cloud12.Transform(transformMatrix12);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 w Transformation", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));
    cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.001 * 2.0, 30));

    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;

    //POKUSAJ REZANJA CLOUDA - ideeeeeee
    open3d::geometry::AxisAlignedBoundingBox boxxx;
    boxxx.min_bound_[0] = -0.3; boxxx.min_bound_[1] = -0.3; boxxx.min_bound_[2] = 0.2;
    boxxx.max_bound_[0] = 0.5; boxxx.max_bound_[1] = 0.15; boxxx.max_bound_[2] = 0.7;
    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx);
    open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    cloudPtr1 = cloud1.Crop(boxxx);
    //    cloud1 = cloudPtr1.get();
    cloudPtr2 = cloud12.Crop(boxxx);
    open3d::visualization::DrawGeometries({buu, cloudPtr1, cloudPtr2}, "BoX2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    //    cloud1 = cloudPtr1.get();

    //    open3d::io::WritePointCloudToPLY("rez_1.ply", *cloudPtr1, true);
    //    open3d::io::WritePointCloudToPLY("rez_2.ply", *cloudPtr2, true);
    //    open3d::io::ReadPointCloud("rez_1.ply", cloud1);
    //    open3d::io::ReadPointCloud("rez_2.ply", cloud12);
    cloud1.PaintUniformColor({1, 0.706, 0});
    cloud12.PaintUniformColor({0, 0.651, 0.929});

    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 before ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    /***   ICP   ***/
    double threshold = 0.05;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=500;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

    //    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud1, cloud12,
    //                                                                           threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud1, cloud12,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    cout << "Fitness: " << reg12.fitness_ << endl << endl;
    cout << "rmse" << reg12.inlier_rmse_ << endl;

    cloud1.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2 after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    /***   ICP   ***/




    //    open3d::visualization::ViewControl pogled;
    //    pogled.ChangeFieldOfView(35);
    //    open3d::visualization::SelectionPolygon::CropPointCloud(cloud1, pogled);
    //    open3d::visualization::DrawGeometries({cloudPtr1}, "Cloud 1 & 2 after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    //    open3d::visualization::SelectionPolygonVolume poligon;
    //    poligon.axis_max_=100.0;
    //    poligon.axis_min_=200.0;
    //    cloudPtr1 = poligon.CropPointCloud(cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr1}, "Crop", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    /********* BOUNDING BOX IZGLEDA KUL  **********/
    //    open3d::geometry::AxisAlignedBoundingBox boxxx;
    //    boxxx.min_bound_[0] = -0.3; boxxx.min_bound_[1] = 0.0; boxxx.min_bound_[2] = -0.1;
    //    boxxx.max_bound_[0] = 0.1; boxxx.max_bound_[1] = 0.2; boxxx.max_bound_[2] = 0.1;
    //    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    //    Eigen::Vector3d min, max;
    //    min[0] =
    //    boxxx = cloud1.GetAxisAlignedBoundingBox();
    //    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    //    boxxx.max_bound_[0] = 100; boxxx.max_bound_[1] = 100; boxxx.max_bound_[2] = 100;
    //    boxxx.min_bound_[0] = 200; boxxx.min_bound_[1] = 200; boxxx.min_bound_[2] = 200;
    //    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::visualization::DrawGeometries({buu, cloudPtr1}, "BoX", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    // OVO samo inverted mi treba
    //    boxxx.min_bound_[0] = -0.3; boxxx.min_bound_[1] = 0.15; boxxx.min_bound_[2] = 0.2;
    //    boxxx.max_bound_[0] = 0.5; boxxx.max_bound_[1] = 0.4; boxxx.max_bound_[2] = 0.7;
    //    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    //    buu = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::visualization::DrawGeometries({buu, cloudPtr1}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    /***** ovo je bounding box za !PRVU! kameru, ma za sve kamere moze *****/
    //    boxxx.min_bound_[0] = -0.25; boxxx.min_bound_[1] = -0.2; boxxx.min_bound_[2] = 0.2;
    //    boxxx.max_bound_[0] = 0.30; boxxx.max_bound_[1] = 0.13; boxxx.max_bound_[2] = 0.6;
    //    boxxx.color_[0] = 1; boxxx.color_[1] = 0; boxxx.color_[2] = 0;
    //    buu = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::visualization::DrawGeometries({buu, cloudPtr1}, "BoX1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    /********** TLOCRT
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "TLOCRT", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    **********/

    /********** NACRT
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.0232;
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -4.05125; front[2] = -4.33795;   // drugi parametar pomiče gore dolje, što veći minus do je više, i treći se nešto može povećavati
    zoom = 0.0212;   // i zoom mi približava i udaljava
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "NACRT", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    **********/
}
void bpoksuho(int, void*)
{
                    // TU JE SADA PRIKAZAN PROBLEM STEREOKALIBRACIJE ZA DOBIVANJE CIJELOG 3D MODELA
    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3, point_cloud4, point_cloud5, point_cloud6, point_cloud7, point_cloud8, point_cloud9, point_cloud10, pc_sum;
    open3d::geometry::PointCloud point_cloud11, point_cloud12, point_cloud13, point_cloud14, point_cloud15, point_cloud16, point_cloud17, point_cloud18, point_cloud19, point_cloud20;
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;
    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;
//    open3d::io::ReadPointCloud("cloud1_br_5.ply", point_cloud1);
//    open3d::io::ReadPointCloud("cloud1_br_7.ply", point_cloud2);
//    open3d::io::ReadPointCloud("cloud1_br_9.ply", point_cloud3);
//    open3d::io::ReadPointCloud("cloud1_br_11.ply", point_cloud4);
//    open3d::io::ReadPointCloud("cloud1_br_13.ply", point_cloud5);
//    open3d::io::ReadPointCloud("cloud1_br_15.ply", point_cloud6);
//    open3d::io::ReadPointCloud("cloud1_br_17.ply", point_cloud7);
//    open3d::io::ReadPointCloud("cloud1_br_19.ply", point_cloud8);
//    open3d::io::ReadPointCloud("cloud1_br_21.ply", point_cloud9);
//    open3d::io::ReadPointCloud("cloud1_br_23.ply", point_cloud10);
//    open3d::io::ReadPointCloud("cloud1_br_25.ply", point_cloud11);
//    open3d::io::ReadPointCloud("cloud1_br_27.ply", point_cloud12);
//    open3d::io::ReadPointCloud("cloud1_br_29.ply", point_cloud13);
//    open3d::io::ReadPointCloud("cloud1_br_31.ply", point_cloud14);
//    open3d::io::ReadPointCloud("cloud1_br_33.ply", point_cloud15);
//    open3d::io::ReadPointCloud("cloud1_br_35.ply", point_cloud16);
//    open3d::io::ReadPointCloud("cloud1_br_37.ply", point_cloud17);
//    open3d::io::ReadPointCloud("cloud1_br_39.ply", point_cloud18);
//    open3d::io::ReadPointCloud("cloud1_br_41.ply", point_cloud19);
//    open3d::io::ReadPointCloud("cloud1_br_43.ply", point_cloud20);

    open3d::io::ReadPointCloud("cede_tris_1.ply", point_cloud1);
    open3d::io::ReadPointCloud("cede_tris_2.ply", point_cloud2);
    open3d::io::ReadPointCloud("cede_tris_3.ply", point_cloud3);
//    point_cloud1.Transform(transformMatrix13);
//    point_cloud2.Transform(transformMatrix13);
//    point_cloud2.Transform(transformMatrix13);
//    point_cloud2.PaintUniformColor({1, 0.706, 0});
//    point_cloud3.PaintUniformColor({0, 0.651, 0.929});
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_4.ply", point_cloud4);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_5.ply", point_cloud5);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_6.ply", point_cloud6);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_7.ply", point_cloud7);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_8.ply", point_cloud8);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_9.ply", point_cloud9);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_10.ply", point_cloud10);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_11.ply", point_cloud11);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_12.ply", point_cloud12);
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_13.ply", point_cloud13);


    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud4);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr5 = std::make_shared<open3d::geometry::PointCloud>(point_cloud5);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr6 = std::make_shared<open3d::geometry::PointCloud>(point_cloud6);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud7);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud8);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr9 = std::make_shared<open3d::geometry::PointCloud>(point_cloud9);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr10 = std::make_shared<open3d::geometry::PointCloud>(point_cloud10);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr11 = std::make_shared<open3d::geometry::PointCloud>(point_cloud11);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr12 = std::make_shared<open3d::geometry::PointCloud>(point_cloud12);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr13 = std::make_shared<open3d::geometry::PointCloud>(point_cloud13);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr14 = std::make_shared<open3d::geometry::PointCloud>(point_cloud14);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr15 = std::make_shared<open3d::geometry::PointCloud>(point_cloud15);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr16 = std::make_shared<open3d::geometry::PointCloud>(point_cloud16);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr17 = std::make_shared<open3d::geometry::PointCloud>(point_cloud17);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr18 = std::make_shared<open3d::geometry::PointCloud>(point_cloud18);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr19 = std::make_shared<open3d::geometry::PointCloud>(point_cloud19);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr20 = std::make_shared<open3d::geometry::PointCloud>(point_cloud20);
//    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3, cloudPtr4, cloudPtr5, cloudPtr6, cloudPtr7, cloudPtr8, cloudPtr9, cloudPtr10,
//                                          cloudPtr11, cloudPtr12, cloudPtr13, cloudPtr14, cloudPtr15, cloudPtr16, cloudPtr17, cloudPtr18, cloudPtr19, cloudPtr20}, "Svih 20 cloudova", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "Svih 20 cloudova", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    point_cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    point_cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    point_cloud3.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud4.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud5.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud6.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud7.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud8.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud9.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud10.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud11.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud13.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud14.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud15.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud16.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud17.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud18.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud19.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//    point_cloud20.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));

    /***   ICP   ***/
    double threshold = 0.004;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    crit.max_iteration_=100;
    crit.relative_fitness_=1.00000;
    crit.relative_rmse_=1.000000e-06;
    open3d::pipelines::registration::RegistrationResult reg1 = open3d::pipelines::registration::RegistrationICP(point_cloud2, point_cloud1,
                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
    open3d::pipelines::registration::RegistrationResult reg2 = open3d::pipelines::registration::RegistrationICP(point_cloud3, point_cloud2,
                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg3 = open3d::pipelines::registration::RegistrationICP(point_cloud4, point_cloud3,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg4 = open3d::pipelines::registration::RegistrationICP(point_cloud5, point_cloud4,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg5 = open3d::pipelines::registration::RegistrationICP(point_cloud6, point_cloud5,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg6 = open3d::pipelines::registration::RegistrationICP(point_cloud7, point_cloud6,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg7 = open3d::pipelines::registration::RegistrationICP(point_cloud8, point_cloud7,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg8 = open3d::pipelines::registration::RegistrationICP(point_cloud9, point_cloud8,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg9 = open3d::pipelines::registration::RegistrationICP(point_cloud10, point_cloud9,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg10 = open3d::pipelines::registration::RegistrationICP(point_cloud11, point_cloud10,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPoint(), crit);
//    open3d::pipelines::registration::RegistrationResult reg11 = open3d::pipelines::registration::RegistrationICP(point_cloud12, point_cloud11,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(point_cloud13, point_cloud12,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg13 = open3d::pipelines::registration::RegistrationICP(point_cloud14, point_cloud13,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg14 = open3d::pipelines::registration::RegistrationICP(point_cloud15, point_cloud14,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg15 = open3d::pipelines::registration::RegistrationICP(point_cloud16, point_cloud15,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg16 = open3d::pipelines::registration::RegistrationICP(point_cloud17, point_cloud16,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg17 = open3d::pipelines::registration::RegistrationICP(point_cloud18, point_cloud17,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg18 = open3d::pipelines::registration::RegistrationICP(point_cloud19, point_cloud18,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//    open3d::pipelines::registration::RegistrationResult reg19 = open3d::pipelines::registration::RegistrationICP(point_cloud20, point_cloud19,
//                                                                                                                threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);


    /*** ICP ***/
    cout << "fitness1: " << reg1.fitness_ << endl;
    cout << "fitness2: " << reg2.fitness_ << endl;
//    cout << "fitness3: " << reg3.fitness_ << endl;
//    cout << "fitness4: " << reg4.fitness_ << endl;
//    cout << "fitness5: " << reg5.fitness_ << endl;
//    cout << "fitness6: " << reg6.fitness_ << endl;
//    cout << "fitness7: " << reg7.fitness_ << endl;
//    cout << "fitness8: " << reg8.fitness_ << endl;
//    cout << "fitness9: " << reg9.fitness_ << endl;
//    cout << "fitness10: " << reg10.fitness_ << endl;
//    cout << "fitness11: " << reg11.fitness_ << endl;
//    cout << "fitness12: " << reg12.fitness_ << endl;
//    cout << "fitness13: " << reg13.fitness_ << endl;
//    cout << "fitness14: " << reg14.fitness_ << endl;
//    cout << "fitness15: " << reg15.fitness_ << endl;
//    cout << "fitness16: " << reg16.fitness_ << endl;
//    cout << "fitness17: " << reg17.fitness_ << endl;
//    cout << "fitness18: " << reg18.fitness_ << endl;
//    cout << "fitness19: " << reg19.fitness_ << endl;
    cout << "rms1: " << reg1.inlier_rmse_ << endl;
    cout << "rms2: " << reg2.inlier_rmse_ << endl;
//    cout << "rms3: " << reg3.inlier_rmse_ << endl;
//    cout << "rms4: " << reg4.inlier_rmse_ << endl;
//    cout << "rms5: " << reg5.inlier_rmse_ << endl;
//    cout << "rms6: " << reg6.inlier_rmse_ << endl;
//    cout << "rms7: " << reg7.inlier_rmse_ << endl;
//    cout << "rms8: " << reg8.inlier_rmse_ << endl;
//    cout << "rms9: " << reg9.inlier_rmse_ << endl;
//    cout << "rms10: " << reg10.inlier_rmse_ << endl;
//    cout << "rms11: " << reg11.inlier_rmse_ << endl;
//    cout << "rms12: " << reg12.inlier_rmse_ << endl;
//    cout << "rms13: " << reg13.inlier_rmse_ << endl;
//    cout << "rms14: " << reg14.inlier_rmse_ << endl;
//    cout << "rms15: " << reg15.inlier_rmse_ << endl;
//    cout << "rms16: " << reg16.inlier_rmse_ << endl;
//    cout << "rms17: " << reg17.inlier_rmse_ << endl;
//    cout << "rms18: " << reg18.inlier_rmse_ << endl;
//    cout << "rms19: " << reg19.inlier_rmse_ << endl;

    point_cloud2.Transform(reg1.transformation_);
    point_cloud3.Transform(reg2.transformation_);
    point_cloud3.Transform(reg1.transformation_);
//    point_cloud4.Transform(reg3.transformation_);
//    point_cloud4.Transform(reg2.transformation_);
//    point_cloud4.Transform(reg1.transformation_);
//    point_cloud5.Transform(reg4.transformation_);
//    point_cloud5.Transform(reg3.transformation_);
//    point_cloud5.Transform(reg2.transformation_);
//    point_cloud5.Transform(reg1.transformation_);
//    point_cloud6.Transform(reg5.transformation_);
//    point_cloud6.Transform(reg4.transformation_);
//    point_cloud6.Transform(reg3.transformation_);
//    point_cloud6.Transform(reg2.transformation_);
//    point_cloud6.Transform(reg1.transformation_);
//    point_cloud7.Transform(reg6.transformation_);
//    point_cloud7.Transform(reg5.transformation_);
//    point_cloud7.Transform(reg4.transformation_);
//    point_cloud7.Transform(reg3.transformation_);
//    point_cloud7.Transform(reg2.transformation_);
//    point_cloud7.Transform(reg1.transformation_);
//    point_cloud8.Transform(reg7.transformation_);
//    point_cloud8.Transform(reg6.transformation_);
//    point_cloud8.Transform(reg5.transformation_);
//    point_cloud8.Transform(reg4.transformation_);
//    point_cloud8.Transform(reg3.transformation_);
//    point_cloud8.Transform(reg2.transformation_);
//    point_cloud8.Transform(reg1.transformation_);
//    point_cloud9.Transform(reg8.transformation_);
//    point_cloud9.Transform(reg7.transformation_);
//    point_cloud9.Transform(reg6.transformation_);
//    point_cloud9.Transform(reg5.transformation_);
//    point_cloud9.Transform(reg4.transformation_);
//    point_cloud9.Transform(reg3.transformation_);
//    point_cloud9.Transform(reg2.transformation_);
//    point_cloud9.Transform(reg1.transformation_);
//    point_cloud10.Transform(reg9.transformation_);
//    point_cloud10.Transform(reg8.transformation_);
//    point_cloud10.Transform(reg7.transformation_);
//    point_cloud10.Transform(reg6.transformation_);
//    point_cloud10.Transform(reg5.transformation_);
//    point_cloud10.Transform(reg4.transformation_);
//    point_cloud10.Transform(reg3.transformation_);
//    point_cloud10.Transform(reg2.transformation_);
//    point_cloud10.Transform(reg1.transformation_);
//    point_cloud11.Transform(reg10.transformation_);
//    point_cloud11.Transform(reg9.transformation_);
//    point_cloud11.Transform(reg8.transformation_);
//    point_cloud11.Transform(reg7.transformation_);
//    point_cloud11.Transform(reg6.transformation_);
//    point_cloud11.Transform(reg5.transformation_);
//    point_cloud11.Transform(reg4.transformation_);
//    point_cloud11.Transform(reg3.transformation_);
//    point_cloud11.Transform(reg2.transformation_);
//    point_cloud11.Transform(reg1.transformation_);
//    point_cloud12.Transform(reg11.transformation_);
//    point_cloud12.Transform(reg10.transformation_);
//    point_cloud12.Transform(reg9.transformation_);
//    point_cloud12.Transform(reg8.transformation_);
//    point_cloud12.Transform(reg7.transformation_);
//    point_cloud12.Transform(reg6.transformation_);
//    point_cloud12.Transform(reg5.transformation_);
//    point_cloud12.Transform(reg4.transformation_);
//    point_cloud12.Transform(reg3.transformation_);
//    point_cloud12.Transform(reg2.transformation_);
//    point_cloud12.Transform(reg1.transformation_);
//    point_cloud13.Transform(reg12.transformation_);
//    point_cloud13.Transform(reg11.transformation_);
//    point_cloud13.Transform(reg10.transformation_);
//    point_cloud13.Transform(reg9.transformation_);
//    point_cloud13.Transform(reg8.transformation_);
//    point_cloud13.Transform(reg7.transformation_);
//    point_cloud13.Transform(reg6.transformation_);
//    point_cloud13.Transform(reg5.transformation_);
//    point_cloud13.Transform(reg4.transformation_);
//    point_cloud13.Transform(reg3.transformation_);
//    point_cloud13.Transform(reg2.transformation_);
//    point_cloud13.Transform(reg1.transformation_);
//    point_cloud14.Transform(reg13.transformation_);
//    point_cloud14.Transform(reg12.transformation_);
//    point_cloud14.Transform(reg11.transformation_);
//    point_cloud14.Transform(reg10.transformation_);
//    point_cloud14.Transform(reg9.transformation_);
//    point_cloud14.Transform(reg8.transformation_);
//    point_cloud14.Transform(reg7.transformation_);
//    point_cloud14.Transform(reg6.transformation_);
//    point_cloud14.Transform(reg5.transformation_);
//    point_cloud14.Transform(reg4.transformation_);
//    point_cloud14.Transform(reg3.transformation_);
//    point_cloud14.Transform(reg2.transformation_);
//    point_cloud14.Transform(reg1.transformation_);
//    point_cloud15.Transform(reg14.transformation_);
//    point_cloud15.Transform(reg13.transformation_);
//    point_cloud15.Transform(reg12.transformation_);
//    point_cloud15.Transform(reg11.transformation_);
//    point_cloud15.Transform(reg10.transformation_);
//    point_cloud15.Transform(reg9.transformation_);
//    point_cloud15.Transform(reg8.transformation_);
//    point_cloud15.Transform(reg7.transformation_);
//    point_cloud15.Transform(reg6.transformation_);
//    point_cloud15.Transform(reg5.transformation_);
//    point_cloud15.Transform(reg4.transformation_);
//    point_cloud15.Transform(reg3.transformation_);
//    point_cloud15.Transform(reg2.transformation_);
//    point_cloud15.Transform(reg1.transformation_);
//    point_cloud16.Transform(reg15.transformation_);
//    point_cloud16.Transform(reg14.transformation_);
//    point_cloud16.Transform(reg13.transformation_);
//    point_cloud16.Transform(reg12.transformation_);
//    point_cloud16.Transform(reg11.transformation_);
//    point_cloud16.Transform(reg10.transformation_);
//    point_cloud16.Transform(reg9.transformation_);
//    point_cloud16.Transform(reg8.transformation_);
//    point_cloud16.Transform(reg7.transformation_);
//    point_cloud16.Transform(reg6.transformation_);
//    point_cloud16.Transform(reg5.transformation_);
//    point_cloud16.Transform(reg4.transformation_);
//    point_cloud16.Transform(reg3.transformation_);
//    point_cloud16.Transform(reg2.transformation_);
//    point_cloud16.Transform(reg1.transformation_);
//    point_cloud17.Transform(reg16.transformation_);
//    point_cloud17.Transform(reg15.transformation_);
//    point_cloud17.Transform(reg14.transformation_);
//    point_cloud17.Transform(reg13.transformation_);
//    point_cloud17.Transform(reg12.transformation_);
//    point_cloud17.Transform(reg11.transformation_);
//    point_cloud17.Transform(reg10.transformation_);
//    point_cloud17.Transform(reg9.transformation_);
//    point_cloud17.Transform(reg8.transformation_);
//    point_cloud17.Transform(reg7.transformation_);
//    point_cloud17.Transform(reg6.transformation_);
//    point_cloud17.Transform(reg5.transformation_);
//    point_cloud17.Transform(reg4.transformation_);
//    point_cloud17.Transform(reg3.transformation_);
//    point_cloud17.Transform(reg2.transformation_);
//    point_cloud17.Transform(reg1.transformation_);
//    point_cloud18.Transform(reg17.transformation_);
//    point_cloud18.Transform(reg16.transformation_);
//    point_cloud18.Transform(reg15.transformation_);
//    point_cloud18.Transform(reg14.transformation_);
//    point_cloud18.Transform(reg13.transformation_);
//    point_cloud18.Transform(reg12.transformation_);
//    point_cloud18.Transform(reg11.transformation_);
//    point_cloud18.Transform(reg10.transformation_);
//    point_cloud18.Transform(reg9.transformation_);
//    point_cloud18.Transform(reg8.transformation_);
//    point_cloud18.Transform(reg7.transformation_);
//    point_cloud18.Transform(reg6.transformation_);
//    point_cloud18.Transform(reg5.transformation_);
//    point_cloud18.Transform(reg4.transformation_);
//    point_cloud18.Transform(reg3.transformation_);
//    point_cloud18.Transform(reg2.transformation_);
//    point_cloud18.Transform(reg1.transformation_);
//    point_cloud19.Transform(reg18.transformation_);
//    point_cloud19.Transform(reg17.transformation_);
//    point_cloud19.Transform(reg16.transformation_);
//    point_cloud19.Transform(reg15.transformation_);
//    point_cloud19.Transform(reg14.transformation_);
//    point_cloud19.Transform(reg13.transformation_);
//    point_cloud19.Transform(reg12.transformation_);
//    point_cloud19.Transform(reg11.transformation_);
//    point_cloud19.Transform(reg10.transformation_);
//    point_cloud19.Transform(reg9.transformation_);
//    point_cloud19.Transform(reg8.transformation_);
//    point_cloud19.Transform(reg7.transformation_);
//    point_cloud19.Transform(reg6.transformation_);
//    point_cloud19.Transform(reg5.transformation_);
//    point_cloud19.Transform(reg4.transformation_);
//    point_cloud19.Transform(reg3.transformation_);
//    point_cloud19.Transform(reg2.transformation_);
//    point_cloud19.Transform(reg1.transformation_);
//    point_cloud20.Transform(reg19.transformation_);
//    point_cloud20.Transform(reg18.transformation_);
//    point_cloud20.Transform(reg17.transformation_);
//    point_cloud20.Transform(reg16.transformation_);
//    point_cloud20.Transform(reg15.transformation_);
//    point_cloud20.Transform(reg14.transformation_);
//    point_cloud20.Transform(reg13.transformation_);
//    point_cloud20.Transform(reg12.transformation_);
//    point_cloud20.Transform(reg11.transformation_);
//    point_cloud20.Transform(reg10.transformation_);
//    point_cloud20.Transform(reg9.transformation_);
//    point_cloud20.Transform(reg8.transformation_);
//    point_cloud20.Transform(reg7.transformation_);
//    point_cloud20.Transform(reg6.transformation_);
//    point_cloud20.Transform(reg5.transformation_);
//    point_cloud20.Transform(reg4.transformation_);
//    point_cloud20.Transform(reg3.transformation_);
//    point_cloud20.Transform(reg2.transformation_);
//    point_cloud20.Transform(reg1.transformation_);


    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
//    cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud4);
//    cloudPtr5 = std::make_shared<open3d::geometry::PointCloud>(point_cloud5);
//    cloudPtr6 = std::make_shared<open3d::geometry::PointCloud>(point_cloud6);
//    cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud7);
//    cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud8);
//    cloudPtr9 = std::make_shared<open3d::geometry::PointCloud>(point_cloud9);
//    cloudPtr10 = std::make_shared<open3d::geometry::PointCloud>(point_cloud10);
//    cloudPtr11 = std::make_shared<open3d::geometry::PointCloud>(point_cloud11);
//    cloudPtr12 = std::make_shared<open3d::geometry::PointCloud>(point_cloud12);
//    cloudPtr13 = std::make_shared<open3d::geometry::PointCloud>(point_cloud13);
//    cloudPtr14 = std::make_shared<open3d::geometry::PointCloud>(point_cloud14);
//    cloudPtr15 = std::make_shared<open3d::geometry::PointCloud>(point_cloud15);
//    cloudPtr16 = std::make_shared<open3d::geometry::PointCloud>(point_cloud16);
//    cloudPtr17 = std::make_shared<open3d::geometry::PointCloud>(point_cloud17);
//    cloudPtr18 = std::make_shared<open3d::geometry::PointCloud>(point_cloud18);
//    cloudPtr19 = std::make_shared<open3d::geometry::PointCloud>(point_cloud19);
//    cloudPtr20 = std::make_shared<open3d::geometry::PointCloud>(point_cloud20);
//    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3, cloudPtr4, cloudPtr5, cloudPtr6, cloudPtr7, cloudPtr8, cloudPtr9, cloudPtr10,
//                                          cloudPtr11, cloudPtr12, cloudPtr13, cloudPtr14, cloudPtr15, cloudPtr16, cloudPtr17, cloudPtr18, cloudPtr19, cloudPtr20}, "Svih 20 cloudova after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "Svih 20 cloudova", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    pc_sum = point_cloud1;
    pc_sum += point_cloud2;
    pc_sum += point_cloud3;
//    pc_sum += point_cloud4;
//    pc_sum += point_cloud5;
//    pc_sum += point_cloud6;
//    pc_sum += point_cloud7;
//    pc_sum += point_cloud8;
//    pc_sum += point_cloud9;
//    pc_sum += point_cloud10;
//    pc_sum += point_cloud11;
//    pc_sum += point_cloud12;
//    pc_sum += point_cloud13;
//    pc_sum += point_cloud14;
//    pc_sum += point_cloud15;
//    pc_sum += point_cloud16;
//    pc_sum += point_cloud17;
//    pc_sum += point_cloud18;
//    pc_sum += point_cloud19;
//    pc_sum += point_cloud20;
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr21 = std::make_shared<open3d::geometry::PointCloud>(pc_sum);
    open3d::io::WritePointCloudToPLY("zbrojeni_cloudovi.ply", *cloudPtr21, true);
    open3d::visualization::DrawGeometries({cloudPtr21}, "Zbrojeni u jedan cloud", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

}

void bpokall(int, void*)
{

    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3;

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;

    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;

    //    std::array<Eigen::Matrix4d_u, 100> uk_transform;
    std::array<Eigen::Matrix4d_u, 100> transformacije1, transformacije2, transformacije3;
    transformacije1[0] = transformMat;
    transformacije2[0] = transformMat;
    transformacije3[0] = transformMat;
    int j = 1;

    for(int i=2; i<30;i++)
    {

        cloud1.Clear();
        cloud2.Clear();
        cloud3.Clear();
        cloud4.Clear();
        cloud5.Clear();
        cloud6.Clear();
        open3d::io::ReadPointCloud("cloud1_br_" + to_string(i-1) + ".ply", cloud1);
        open3d::io::ReadPointCloud("cloud1_br_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i-1) + ".ply", cloud3);
        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i) + ".ply", cloud4);
        open3d::io::ReadPointCloud("cloud3_br_" + to_string(i-1) + ".ply", cloud5);
        open3d::io::ReadPointCloud("cloud3_br_" + to_string(i) + ".ply", cloud6);

        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud3.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud4.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud5.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud6.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));

        //        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        //        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        /***   ICP   ***/
        double threshold = 0.001;
        cout << "Kreće ICP:" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
        crit.max_iteration_=100;
        crit.relative_fitness_=1.000000;
        crit.relative_rmse_=1.000000e-06;

        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        open3d::pipelines::registration::RegistrationResult reg34 = open3d::pipelines::registration::RegistrationICP(cloud4, cloud3,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        open3d::pipelines::registration::RegistrationResult reg56 = open3d::pipelines::registration::RegistrationICP(cloud6, cloud5,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

        transformacije1[j] = reg12.transformation_;
        transformacije2[j] = reg34.transformation_;
        transformacije3[j] = reg56.transformation_;
        //        cout << "reg transform: " << reg12.transformation_ << endl;
        //        cout << "transformacije: " << transformacije1[j];
        //        cloud2.Transform(transformacije1[j]);
        //        cloud4.Transform(transformacije2[j]);
        //        cloud6.Transform(transformacije3[j]);
        //        cloud1 += cloud2;

        //        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        //        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        cout << "i iznosi: " << i << endl;
        cout << "fitness1 iznosi: " << reg12.fitness_ << endl;
        cout << "fitness2 iznosi: " << reg34.fitness_ << endl;
        cout << "fitness3 iznosi: " << reg56.fitness_ << endl;
        //        i = i + 2;
        j += 1;
    }
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    open3d::io::WritePointCloudToPLY("cloud2_31-40.ply", *cloudPtr1, true);
    //    open3d::visualization::DrawGeometries({cloudPtr1}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    /***** POČINJE UČITAVANJE I TRANSFORMACIJA CLOUDOVA *****/
    point_cloud1.Clear();
    open3d::io::ReadPointCloud("cloud1_br_1.ply", point_cloud1);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    open3d::io::ReadPointCloud("cloud2_br_1.ply", point_cloud2);
    open3d::io::ReadPointCloud("cloud3_br_1.ply", point_cloud3);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr9 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
    //    open3d::visualization::DrawGeometries({cloudPtr3}, "Point_cloud1 smo učitali", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    int k = 1;
    int i;

    for(i = 2; i < 30; i++)
    {
        cloud1.Clear();
        cloud2.Clear();
        cloud3.Clear();
        open3d::io::ReadPointCloud("cloud1_br_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("cloud3_br_" + to_string(i) + ".ply", cloud3);

        for(j = k; j > 0; j--)
        {
            cloud1.Transform(transformacije1[j]);
            cloud2.Transform(transformacije2[j]);
            cloud3.Transform(transformacije3[j]);
            cout << "j jest:" << j << endl;
        }

        point_cloud1 += cloud1;
        point_cloud2 += cloud2;
        point_cloud3 += cloud3;
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        //        i = i + 2;
        k += 1;
    }


    cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud2);
    cloudPtr9 = std::make_shared<open3d::geometry::PointCloud>(point_cloud3);
    open3d::visualization::DrawGeometries({cloudPtr7}, "Prva kamera:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr8}, "Druga kamera:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr9}, "Treca kamera:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    open3d::visualization::DrawGeometries({cloudPtr7, cloudPtr8, cloudPtr9}, "Sve tri prije završnog:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    open3d::io::WritePointCloudToPLY("svi_cam1.ply", *cloudPtr7, true);
    open3d::io::WritePointCloudToPLY("svi_cam2.ply", *cloudPtr8, true);
    open3d::io::WritePointCloudToPLY("svi_cam3.ply", *cloudPtr9, true);

    // onda ih učiati u transform123 i isprobati?

}
void b3T(int, void*)
{

    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3;

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;

    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;

    //    std::array<Eigen::Matrix4d_u, 100> uk_transform;
    std::array<Eigen::Matrix4d_u, 100> transformacije1, transformacije2, transformacije3;
    transformacije1[0] = transformMat;
    transformacije2[0] = transformMat;
    transformacije3[0] = transformMat;
    int j = 1;

    for(int i=1; i<58;i++)
    {

        cloud1.Clear();
        cloud2.Clear();
        cloud3.Clear();
        point_cloud1.Clear();
        open3d::io::ReadPointCloud("cloud1_br_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("cloud3_br_" + to_string(i) + ".ply", cloud3);

        cloud2.Transform(transformMatrix12);
        cloud3.Transform(transformMatrix13);

        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud3.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "Cloud 1, 2 & 3", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        /***   ICP   ***/
        double threshold = 0.008;
        cout << "Kreće ICP:" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
        crit.max_iteration_=100;
        crit.relative_fitness_=1.000000;
        crit.relative_rmse_=1.000000e-06;

        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        open3d::pipelines::registration::RegistrationResult reg13 = open3d::pipelines::registration::RegistrationICP(cloud3, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

        //        transformacije1[j] = reg12.transformation_;
        //        transformacije2[j] = reg13.transformation_;
                cout << "reg transform: " << reg12.transformation_ << endl;
        //        cout << "transformacije: " << transformacije1[j];
        cloud2.Transform(reg12.transformation_);
        cloud3.Transform(reg13.transformation_);

        point_cloud1 += cloud1;
        point_cloud1 += cloud2;
        point_cloud1 += cloud3;

        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
        open3d::visualization::DrawGeometries({cloudPtr4}, "Cloud 1, 2 & 3 after ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        open3d::io::WritePointCloudToPLY("tris_" + to_string(j) + ".ply", *cloudPtr4, true);

        //        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        cout << "i iznosi: " << i << endl;
        cout << "fitness1 iznosi: " << reg12.fitness_ << endl;
        cout << "fitness2 iznosi: " << reg13.fitness_ << endl;
        i = i + 3;
        j += 1;
    }



    int k = 1;
    j = 1;
    int i;

    for(i = 2; i < 16; i++)
    {
        cloud1.Clear();
        cloud2.Clear();
        open3d::io::ReadPointCloud("tris_" + to_string(i-1) + ".ply", cloud1);
        open3d::io::ReadPointCloud("tris_" + to_string(i) + ".ply", cloud2);

        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        /***   ICP   ***/
        double threshold = 0.005;
        cout << "Kreće ICP:" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
* relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
        crit.max_iteration_=100;
        crit.relative_fitness_=1.000000;
        crit.relative_rmse_=1.000000e-06;

        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

        transformacije1[j] = reg12.transformation_;
        //        cout << "reg transform: " << reg12.transformation_ << endl;
        //        cout << "transformacije: " << transformacije1[j];
        cloud2.Transform(transformacije1[j]);
        //        cloud1 += cloud2;
        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        cout << "i iznosi: " << i << endl;
        cout << "fitness1 iznosi: " << reg12.fitness_ << endl;
        //        i = i + 2;
        j += 1;
    }



    /***** POČINJE UČITAVANJE I TRANSFORMACIJA CLOUDOVA *****/
    point_cloud1.Clear();
    open3d::io::ReadPointCloud("tris_1.ply", point_cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr3}, "Point_cloud1 smo učitali", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    k = 1;
    for(i = 2; i < 16; i++)
    {
        cloud1.Clear();
        open3d::io::ReadPointCloud("tris_" + to_string(i) + ".ply", cloud1);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        for(j = k; j > 0; j--)
        {
            cloud1.Transform(transformacije1[j]);
            cout << "j jest:" << j << endl;
        }

        point_cloud1 += cloud1;
        cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        //        i = i + 2;
        k += 1;
    }

    //    cloudPtr7 = point_cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
    cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    open3d::visualization::DrawGeometries({cloudPtr7}, "Sve:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

}


// prvo pronaći boxove da ne odreže cedevitu, zatim napraviti voxel i odrezati i zatim probati ICP s jednom kamerom (može i na poksuho se probat) i onda sve tri(na bcede funkciji) ****************************************
void bsingle_cloud_icp(int,void*)
{
    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3, point_cloud4, point_cloud5, point_cloud6, point_cloud7, pc_sum;
    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;
    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;

//**************************************************** Jedan Dio ****************************************************
//    open3d::io::ReadPointCloud("cloud2_br_1.ply", cloud1);
//    point_cloud1 = cloud1;
//    std::array<Eigen::Matrix4d_u, 100> transformacije1;
//    int j = 1;

//    for(int i=1; i<30; i++)
//    {
//        cloud1.Clear();
//        cloud2.Clear();

//        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i) + ".ply", cloud1);
//        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i+2) + ".ply", cloud2);

//        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);

//        /***   ICP   ***/
//        double threshold = 0.005;
//        cout << "Kreće ICP:" << endl;
//        open3d::pipelines::registration::ICPConvergenceCriteria crit;
//        crit.max_iteration_=100;
//        crit.relative_fitness_=1.00000;
//        crit.relative_rmse_=1.000000e-06;
//        open3d::pipelines::registration::RegistrationResult reg1 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
//                                                                          threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

//        transformacije1[j] = reg1.transformation_;
////        cloud2.Transform(reg1.transformation_);

////        point_cloud1 += cloud2;
//        j = j + 1;
//        i=i+1;

//    }

//    int k = 1;
//    j=0;

//    for(int i = 4; i < 30; i++)
//    {
//        cloud2.Clear();
//        open3d::io::ReadPointCloud("cloud2_br_" + to_string(i) + ".ply", cloud2);

//        for(j = k; j > 0; j--)
//        {
//            cloud2.Transform(transformacije1[j]);
//            cout << "j jest:" << j << endl;
//        }

//        point_cloud1 += cloud2;
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
//        //        i = i + 2;
//        k += 1;
//        i=i+1;
//    }

//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//    open3d::visualization::DrawGeometries({cloudPtr8}, "zbrojeni u jedan cloud", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
//**************************************************** Jedan Dio ****************************************************

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1, cloudPtr2, cloudPtr3;


//**************************************************** ICP Cedevita ****************************************************
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_002_1.ply", cloud1);
//    point_cloud1 = cloud1;
//    std::array<Eigen::Matrix4d_u, 100> transformacije1;
//    int j = 1;

//    for(int i=1; i<13; i++)
//    {
//        cloud1.Clear();
//        cloud2.Clear();

//        open3d::io::ReadPointCloud("3D_cedevita_cam1_002_" + to_string(i) + ".ply", cloud1);
//        open3d::io::ReadPointCloud("3D_cedevita_cam1_002_" + to_string(i+1) + ".ply", cloud2);


//        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
//        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);

//        /***   ICP   ***/
//        double threshold = 0.005;
//        cout << "Kreće ICP:" << endl;
//        open3d::pipelines::registration::ICPConvergenceCriteria crit;
//        crit.max_iteration_=50;
//        crit.relative_fitness_=1.00000;
//        crit.relative_rmse_=1.000000e-06;
//        open3d::pipelines::registration::RegistrationResult reg1 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
//                                                                          threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

//        transformacije1[j] = reg1.transformation_;
////        cloud2.Transform(reg1.transformation_);
////        point_cloud1 += cloud2;
//        j = j + 1;

//        cout <<"tu" << endl;
//    }


//    int k = 1;
//    j=0;

//    cout <<"tu" << endl;
//    for(int i = 2; i < 13; i++)
//    {
//        cloud2.Clear();
//        open3d::io::ReadPointCloud("3D_cedevita_cam1_002_" + to_string(i) + ".ply", cloud2);

//        for(j = k; j > 0; j--)
//        {
//            cloud2.Transform(transformacije1[j]);
//            cout << "j jest:" << j << endl;
//        }

//        point_cloud1 += cloud2;
//        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
//        //        i = i + 2;
//        k += 1;
//    }

//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr8 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
//    open3d::visualization::DrawGeometries({cloudPtr8}, "zbrojeni u jedan cloud", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
//**************************************************** ICP Cedevita ****************************************************



//**************************************************** Sređivanje cloudova ****************************************************
    open3d::geometry::AxisAlignedBoundingBox boxxx1, boxxx2, boxxx3;   // boxevi za cedevitu
    boxxx1.min_bound_[0] = -0.17; boxxx1.min_bound_[1] = -0.15; boxxx1.min_bound_[2] = 0.2;
    boxxx1.max_bound_[0] = 0.20; boxxx1.max_bound_[1] = 0.13; boxxx1.max_bound_[2] = 0.5;
    boxxx1.color_[0] = 1; boxxx1.color_[1] = 0; boxxx1.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu1 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx1);

    boxxx2.min_bound_[0] = -0.15; boxxx2.min_bound_[1] = -0.15; boxxx2.min_bound_[2] = 0.15;
    boxxx2.max_bound_[0] = 0.2; boxxx2.max_bound_[1] = 0.135; boxxx2.max_bound_[2] = 0.45;
    boxxx2.color_[0] = 1; boxxx2.color_[1] = 0; boxxx2.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu2 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx2);

    boxxx3.min_bound_[0] = -0.18; boxxx3.min_bound_[1] = -0.15; boxxx3.min_bound_[2] = 0.15;
    boxxx3.max_bound_[0] = 0.13; boxxx3.max_bound_[1] = 0.135; boxxx3.max_bound_[2] = 0.46;
    boxxx3.color_[0] = 1; boxxx3.color_[1] = 0; boxxx3.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu3 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx3);


    for(int i=1;i<8;i++){

        cloud1.Clear();cloud2.Clear(); cloud3.Clear();
        open3d::io::ReadPointCloud("3D_cedevita_cam1_first_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("3D_cedevita_cam2_first_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("3D_cedevita_cam3_first_" + to_string(i) + ".ply", cloud3);

        cloudPtr1 = cloud1.VoxelDownSample(0.002); //  t = 0.00780749 s
        cloudPtr2 = cloud2.VoxelDownSample(0.002); //  t = 0.00780749 s
        cloudPtr3 = cloud3.VoxelDownSample(0.002); //  t = 0.00780749 s

        cloudPtr1 = cloud1.Crop(boxxx1);
        cloudPtr2 = cloud2.Crop(boxxx2);
        cloudPtr3 = cloud3.Crop(boxxx3);

        open3d::io::WritePointCloudToPLY("3D_cedevita_cam1_002_" + to_string(i) + ".ply", *cloudPtr1, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
        open3d::io::WritePointCloudToPLY("3D_cedevita_cam2_002_" + to_string(i) + ".ply", *cloudPtr2, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
        open3d::io::WritePointCloudToPLY("3D_cedevita_cam3_002_" + to_string(i) + ".ply", *cloudPtr3, true);
    }
    cout << "Završeno." << endl;
 //**************************************************** Sređivanje cloudova ****************************************************


//**************************************************** Pronalazak odgovarajućih boxxxova ****************************************************
//    open3d::io::ReadPointCloud("3D_cedevita_cam1_first_5.ply", cloud1);
//    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    open3d::visualization::DrawGeometries({cloudPtr1, buu1}, "cloud cam 1", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

//    open3d::io::ReadPointCloud("3D_cedevita_cam2_first_10.ply", cloud2);
//    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
//    open3d::visualization::DrawGeometries({cloudPtr2, buu2}, "cloud cam 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

//    open3d::io::ReadPointCloud("3D_cedevita_cam3_first_10.ply", cloud3);
//    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
//    open3d::visualization::DrawGeometries({cloudPtr3, buu3}, "cloud cam 3", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
//**************************************************** Pronalazak odgovarajućih boxxxova ****************************************************
}



void bcede(int,void*){

    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3;

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272;
    look[1] = -0.2475;
    look[2] = 0.3232; //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694;
    up[1] = -0.9768;
    up[2] = 0.2024;
    front[0] = -0.02795;
    front[1] = -0.35125;
    front[2] = -0.03795;
    zoom = 0.2212;

    Eigen::Matrix4d transformMat;
    transformMat.setZero(4, 4);
    transformMat(0, 0) = 1;
    transformMat(1, 1) = 1;
    transformMat(2, 2) = 1;
    transformMat(3, 3) = 1;

    //    std::array<Eigen::Matrix4d_u, 100> uk_transform;
    std::array<Eigen::Matrix4d_u, 100> transformacije1, transformacije2, transformacije3;
    transformacije1[0] = transformMat;
    transformacije2[0] = transformMat;
    transformacije3[0] = transformMat;
    int j = 1;

    for (int i = 1; i < 7; i++)
    {

        cout << "iteracija je: " << i << endl;
        cloud1.Clear();
        cloud2.Clear();
        cloud3.Clear();
        point_cloud1.Clear();
        open3d::io::ReadPointCloud("3D_cedevita_cam1_002_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("3D_cedevita_cam2_002_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("3D_cedevita_cam3_002_" + to_string(i) + ".ply", cloud3);

        cloud2.Transform(transformMatrix12);
        cloud3.Transform(transformMatrix13);

        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        //        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud3.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "Cloud 1, 2 & 3 prije ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        /***   ICP   ***/
        double threshold = 0.005;
        cout << "Kreće ICP:" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
         * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
        crit.max_iteration_ = 100;
        crit.relative_fitness_ = 1.000000;
        crit.relative_rmse_ = 1.000000e-06;

        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
        open3d::pipelines::registration::RegistrationResult reg13 = open3d::pipelines::registration::RegistrationICP(cloud3, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

        transformacije1[j] = reg12.transformation_;
        transformacije2[j] = reg13.transformation_;
//        cout << "reg transform12: " << reg12.inlier_rmse_ << endl;
//        cout << "reg transform12: " << reg13.inlier_rmse_ << endl;
        //        cout << "transformacije: " << transformacije1[j];

        open3d::io::ReadPointCloud("3D_cedevita_cam1_002_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("3D_cedevita_cam2_002_" + to_string(i) + ".ply", cloud2);
        open3d::io::ReadPointCloud("3D_cedevita_cam3_002_" + to_string(i) + ".ply", cloud3);
        cloud2.Transform(transformMatrix12);
        cloud3.Transform(transformMatrix13);
        cloud2.Transform(reg12.transformation_);
        cloud3.Transform(reg13.transformation_);
        point_cloud1 = cloud1;
        point_cloud1 += cloud2;
        point_cloud1 += cloud3;
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
        open3d::visualization::DrawGeometries({cloudPtr4}, "Cloud 1, 2 & 3 after ICP tris broj: " + to_string(i) , 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        open3d::io::WritePointCloudToPLY("cede_tris_" + to_string(i) + ".ply", *cloudPtr4, true);
        //        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        cout << "i iznosi: " << i << endl;
        cout << "fitness1 iznosi: " << reg12.fitness_ << endl;
        cout << "fitness2 iznosi: " << reg13.fitness_ << endl;
        cout << "rmse1 iznosi: " << reg12.inlier_rmse_ << endl; // provjeriti jel je rmse
        cout << "rmse2 iznosi: " << reg13.inlier_rmse_ << endl << endl;
        j += 1;
    }

    int k = 1;
    j = 1;
    int i;

    for (i = 1; i < 6; i++)
    {
        cloud1.Clear();
        cloud2.Clear();
        open3d::io::ReadPointCloud("cede_tris_" + to_string(i) + ".ply", cloud1);
        open3d::io::ReadPointCloud("cede_tris_" + to_string(i + 1) + ".ply", cloud2);

        cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        cloud2.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Cloud 1 & 2", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
        /***   ICP   ***/
        double threshold = 0.005;
        cout << "Kreće ICP:" << endl;
        open3d::pipelines::registration::ICPConvergenceCriteria crit;
        /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
         * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
        crit.max_iteration_ = 100;
        crit.relative_fitness_ = 1.000000;
        crit.relative_rmse_ = 1.000000e-06;

        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud2, cloud1,
                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

        transformacije3[j] = reg12.transformation_;
        //        cout << "reg transform: " << reg12.transformation_ << endl;
        //        cout << "transformacije: " << transformacije1[j];
        cloud2.Transform(transformacije1[j]);
        //        cloud1 += cloud2;
        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
        //        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "After ICP", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        cout << "i iznosi: " << i << endl;
        cout << "fitness1 iznosi: " << reg12.fitness_ << endl;
        j += 1;
    }

    /***** POČINJE UČITAVANJE I TRANSFORMACIJA CLOUDOVA *****/
    point_cloud1.Clear();
    open3d::io::ReadPointCloud("cede_tris_1.ply", point_cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    open3d::visualization::DrawGeometries({cloudPtr3}, "Point_cloud1 smo učitali", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

    k = 1;
    for (i = 2; i < 7; i++)
    {
        cloud1.Clear();
        open3d::io::ReadPointCloud("cede_tris_" + to_string(i) + ".ply", cloud1);
        // std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
        // open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        for (j = k; j > 0; j--)
        {
            cloud1.Transform(transformacije3[j]);
            cout << "j jest:" << j << endl;
        }

        point_cloud1 += cloud1;
        std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
        open3d::visualization::DrawGeometries({cloudPtr3}, "Jedan po jedan", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);

        k += 1;
    }

    //    cloudPtr7 = point_cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
    cloudPtr7 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    open3d::visualization::DrawGeometries({cloudPtr7}, "Sve:", 1500, 1000, 150, 50, false, false, false, &look, &up, &front, &zoom);
}



void bzuto(int, void*)
{

    //            IZDVAJANJE ŽUTOG IZ SLIKE    PRVO USLIKATI SLIKE PA OTKOMENTIRATI I DOBIVATI ŠTO ŽELIM, prvo uzeti sve ostalo zakomentirati
    Mat img1;
    img1.release();

//    //input1 = fRealsenseImageGrabber1();  2D
//    rs2::frameset frames1 = pipeReal1.wait_for_frames();
//    rs2::video_frame colorFrame=frames1.get_color_frame();
//    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage.clone();
//    imwrite("zuto_2D_BGR.png", img1);

    // 3D
//    rs2::threshold_filter thresholdFilter;
//    frames1 = align_to_depth1.process(frames1);
//    rs2::depth_frame depth1 = frames1.get_depth_frame(); //  t = 0.00348485 s
//    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.25f); //   daljina na kojoj da hvata point cloud
//    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);  //  t = 0.000399798 s
//    depth1 = thresholdFilter.process(depth1);
//    const int w = colorFrame.get_width();  //   da bi dobili dimenzije matrice?
//    const int h = colorFrame.get_height(); //  t = 1.54e-07 s
//    pc1.map_to(colorFrame);
//    auto points1 = pc1.calculate(depth1); //   t = 0.002 s
//    cloud1.Clear();
//    cloud1 = open3d::geometry::PointCloud(fPointsToPc(points1, colorFrame));
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1); //   t = 0.00161014 s
//    open3d::io::WritePointCloudToPLY("zuto_3D_first.ply", *cloudPtr1, true);




/****************************************************** SVE KAMERE ******************************************************/
    // 2D
//    rs2::frameset frames1 = pipeReal1.wait_for_frames();
//    rs2::frameset frames2 = pipeReal2.wait_for_frames();
//    rs2::frameset frames3 = pipeReal3.wait_for_frames();
//    rs2::video_frame colorFrame=frames1.get_color_frame();
//    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage.clone();
//    imwrite("2D_cam1_first.png", img1);
//    // 3D
//    rs2::threshold_filter thresholdFilter;
//    frames1 = align_to_depth1.process(frames1);
//    rs2::depth_frame depth1 = frames1.get_depth_frame(); //  t = 0.00348485 s
//    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.25f); //   daljina na kojoj da hvata point cloud
//    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);  //  t = 0.000399798 s
//    depth1 = thresholdFilter.process(depth1);
//    const int w = colorFrame.get_width();  //   da bi dobili dimenzije matrice?
//    const int h = colorFrame.get_height(); //  t = 1.54e-07 s
//    pc1.map_to(colorFrame);
//    auto points1 = pc1.calculate(depth1); //   t = 0.002 s
//    cloud1.Clear();
//    cloud1 = open3d::geometry::PointCloud(fPointsToPc(points1, colorFrame));
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1); //   t = 0.00161014 s
//    open3d::io::WritePointCloudToPLY("3D_cam1_first.ply", *cloudPtr1, true);

//    // 2D
//    rs2::video_frame colorFrame2=frames2.get_color_frame();
//    cv::Mat colorImage2(cv::Size(colorFrame2.get_width(), colorFrame2.get_height()), CV_8UC3, (void*)colorFrame2.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage2.clone();
//    imwrite("2D_cam2_first.png", img1);
//    // 3D
//    frames2 = align_to_depth2.process(frames2);
//    rs2::depth_frame depth2 = frames2.get_depth_frame(); //  t = 0.00348485 s
//    depth2 = thresholdFilter.process(depth2);
//    pc2.map_to(colorFrame2);
//    auto points2 = pc2.calculate(depth2); //   t = 0.002 s
//    cloud2.Clear();
//    cloud2 = open3d::geometry::PointCloud(fPointsToPc(points2, colorFrame2));
//    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud2); //   t = 0.00161014 s
//    open3d::io::WritePointCloudToPLY("3D_cam2_first.ply", *cloudPtr1, true);

//    // 2D
//    rs2::video_frame colorFrame3=frames3.get_color_frame();
//    cv::Mat colorImage3(cv::Size(colorFrame3.get_width(), colorFrame3.get_height()), CV_8UC3, (void*)colorFrame3.get_data(), cv::Mat::AUTO_STEP);
//    img1 = colorImage3.clone();
//    imwrite("2D_cam3_first.png", img1);
//    // 3D
//    frames3 = align_to_depth3.process(frames3);
//    rs2::depth_frame depth3 = frames3.get_depth_frame(); //  t = 0.00348485 s
//    depth3 = thresholdFilter.process(depth3);
//    pc3.map_to(colorFrame3);
//    auto points3 = pc3.calculate(depth3); //   t = 0.002 s
//    cloud3.Clear();
//    cloud3 = open3d::geometry::PointCloud(fPointsToPc(points3, colorFrame3));
//    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud3); //   t = 0.00161014 s
//    open3d::io::WritePointCloudToPLY("3D_cam3_first.ply", *cloudPtr1, true);


/************************************** UZIMANJE CEDEVITE **************************************/ // jedina razlika je u spremanju cloudova
    rs2::frameset frames1 = pipeReal1.wait_for_frames();
    rs2::frameset frames2 = pipeReal2.wait_for_frames();
    rs2::frameset frames3 = pipeReal3.wait_for_frames();
    rs2::video_frame colorFrame=frames1.get_color_frame();
    cv::Mat colorImage(cv::Size(colorFrame.get_width(), colorFrame.get_height()), CV_8UC3, (void*)colorFrame.get_data(), cv::Mat::AUTO_STEP);
    img1 = colorImage.clone();
    imwrite("2D_cedevita_cam1_first_" + to_string(cede)+".png", img1);
    // 3D
    rs2::threshold_filter thresholdFilter;
    frames1 = align_to_depth1.process(frames1);
    rs2::depth_frame depth1 = frames1.get_depth_frame(); //  t = 0.00348485 s
    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.25f); //   daljina na kojoj da hvata point cloud
    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 0.6f);  //  t = 0.000399798 s
    depth1 = thresholdFilter.process(depth1);
    const int w = colorFrame.get_width();  //   da bi dobili dimenzije matrice?
    const int h = colorFrame.get_height(); //  t = 1.54e-07 s
    pc1.map_to(colorFrame);
    auto points1 = pc1.calculate(depth1); //   t = 0.002 s
    cloud1.Clear();
    cloud1 = open3d::geometry::PointCloud(fPointsToPc(points1, colorFrame));
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1); //   t = 0.00161014 s
    open3d::io::WritePointCloudToPLY("3D_cedevita_cam1_first_" + to_string(cede)+".ply", *cloudPtr1, true);

    // 2D
    rs2::video_frame colorFrame2=frames2.get_color_frame();
    cv::Mat colorImage2(cv::Size(colorFrame2.get_width(), colorFrame2.get_height()), CV_8UC3, (void*)colorFrame2.get_data(), cv::Mat::AUTO_STEP);
    img1 = colorImage2.clone();
    imwrite("2D_cedevita_cam2_first_" + to_string(cede)+".png", img1);
    // 3D
    frames2 = align_to_depth2.process(frames2);
    rs2::depth_frame depth2 = frames2.get_depth_frame(); //  t = 0.00348485 s
    depth2 = thresholdFilter.process(depth2);
    pc2.map_to(colorFrame2);
    auto points2 = pc2.calculate(depth2); //   t = 0.002 s
    cloud2.Clear();
    cloud2 = open3d::geometry::PointCloud(fPointsToPc(points2, colorFrame2));
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud2); //   t = 0.00161014 s
    open3d::io::WritePointCloudToPLY("3D_cedevita_cam2_first_" + to_string(cede)+".ply", *cloudPtr1, true);

    // 2D
    rs2::video_frame colorFrame3=frames3.get_color_frame();
    cv::Mat colorImage3(cv::Size(colorFrame3.get_width(), colorFrame3.get_height()), CV_8UC3, (void*)colorFrame3.get_data(), cv::Mat::AUTO_STEP);
    img1 = colorImage3.clone();
    imwrite("2D_cedevita_cam3_first_" + to_string(cede)+".png", img1);
    // 3D
    frames3 = align_to_depth3.process(frames3);
    rs2::depth_frame depth3 = frames3.get_depth_frame(); //  t = 0.00348485 s
    depth3 = thresholdFilter.process(depth3);
    pc3.map_to(colorFrame3);
    auto points3 = pc3.calculate(depth3); //   t = 0.002 s
    cloud3.Clear();
    cloud3 = open3d::geometry::PointCloud(fPointsToPc(points3, colorFrame3));
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud3); //   t = 0.00161014 s
    open3d::io::WritePointCloudToPLY("3D_cedevita_cam3_first_" + to_string(cede)+".ply", *cloudPtr1, true);

    cede = cede + 1;

/***************************************************/







//    // 1.cam : R=110,255 ; G=30,255 ; B=12,36
//    // 2.cam : R=100,255 ; G=50,255 ; B=14,36
//    // 3.cam : R=110,255 ; G=30,255 ; B=15,36

//    Mat frame, frame_threshold;
////    frame = imread("zuto_2D_BGR.png");
//    frame = imread("2D_cam3_first.png");
////    imwrite("zuto_2D_first.png", frame);
//    imshow("Output3", frame);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
//    Mat hsv_image1;
//    cvtColor(frame, hsv_image1, COLOR_BGR2HSV);
////    imwrite("zuto_2D_HSV.png", hsv_image1);
//    inRange(hsv_image1,Scalar(low_b,low_g,low_r), Scalar(high_b,high_g,high_r),frame_threshold);
//    bitwise_not(frame_threshold, frame_threshold);
////    imshow("Output2", frame_threshold);
////    imwrite("zuto_2D_thresh.png", frame_threshold);


//    Mat img_bw;
//    cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
////    cv::imwrite("image_bw.png", img_bw);

//    cvtColor(frame_threshold,frame_threshold,COLOR_GRAY2BGR);
//    bitwise_and(frame, frame_threshold,frame);
//    imshow("Output2", frame);
////    cv::imwrite("zuto_2D_result.png", frame);
////    imshow("Output2", frame_threshold);
////    imshow("Output1", frame);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
//    imshow("Output1", img_bw);
////    imwrite("zuto_2D_binary.png", img_bw);

//    waitKey(0);

//    Mat m_flat = img_bw.reshape(1,1); // flat 1d
//    std::vector <int> veki;
//    std::vector <long unsigned int> rez_vect;
//    veki = m_flat.row(0);


//    for (int i = 0; i < veki.size(); i++) {
//        if(veki.at(i) == 255){
//            veki.at(i) = 1;
//            veki.at(i) = veki.at(i) * i;
//            rez_vect.push_back(veki.at(i));
//        }
////        veki.at(i) = veki.at(i) * i;
////        std::cout << veki.at(i) << ' ';
//    }
////    std::cout << "veki size: " << veki.size() << endl;
////    veki.erase(std::remove(veki.begin(), veki.end(), 0),veki.end());
////    veki.shrink_to_fit();

////    for (int i = 0; i < veki.size(); i++) {
////        rez_vect.push_back(veki.at(i));
////        std::cout << rez_vect.at(i) << ' ';
////    }
//    std::cout << "veki size: " << veki.size() << endl;
//    std::cout << "rez_vect size: " << rez_vect.size() << endl;


//    open3d::geometry::AxisAlignedBoundingBox boxxx1;
//    boxxx1.min_bound_[0] = -0.17; boxxx1.min_bound_[1] = -0.15; boxxx1.min_bound_[2] = 0.2;
//    boxxx1.max_bound_[0] = 0.20; boxxx1.max_bound_[1] = 0.13; boxxx1.max_bound_[2] = 0.5;
//    boxxx1.color_[0] = 1; boxxx1.color_[1] = 0; boxxx1.color_[2] = 0;
//    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu1 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx1);

////    open3d::io::ReadPointCloud("zuti_cloud3.ply", cloud1);
//    open3d::io::ReadPointCloud("zuto_3D_first.ply", cloud1);
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    open3d::visualization::DrawGeometries({cloudPtr1}, "zuti_cloud", 1500, 1000, 150, 50);
//    open3d::visualization::DrawGeometries({cloudPtr1, buu1}, "zuti_cloud+BoundingBox", 1500, 1000, 150, 50);



//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    cloudPtr2 = cloud1.SelectByIndex(rez_vect);
////    cloudPtr2.get()->SelectByIndex(rez_vect);
////    cloudPtr2.get()->Crop(boxxx2);
////    cloudPtr2.get()->VoxelDownSample(0.002);
////    cloudPtr2.get()->Crop(boxxx2);
////    cloudPtr2 = cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
//    open3d::visualization::DrawGeometries({cloudPtr2}, "zuti_cloud_inRange", 1500, 1000, 150, 50);
//    open3d::io::WritePointCloudToPLY("zuto_3D_second.ply", *cloudPtr2, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//    open3d::io::ReadPointCloud("zuto_3D_second.ply", cloud1);
//    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//    cloudPtr2 = cloud1.VoxelDownSample(0.002);   //  t = 0.00780749 s
//    open3d::io::WritePointCloudToPLY("zuto_3D_Voxel.ply", *cloudPtr2, true);
//    open3d::visualization::DrawGeometries({cloudPtr2, buu1}, "zuti_cloud+BoundingBox", 1500, 1000, 150, 50);
//    cloudPtr2 = cloud1.Crop(boxxx1);
//    open3d::visualization::DrawGeometries({cloudPtr2}, "zuti_cloud_third", 1500, 1000, 150, 50);
////    cloudPtr1 = cloud1.Crop(boxxx2);
////    open3d::visualization::DrawGeometries({cloudPtr1}, "cloud_finale", 1500, 1000, 150, 50);
//    open3d::io::WritePointCloudToPLY("zuto_3D_final.ply", *cloudPtr2, true);

////    cloudPtr2 = cloud1.SelectByIndex(rez_vect2);
////    cloudPtr2 = cloud2.VoxelDownSample(0.002);   //  t = 0.00780749 s
////    cloudPtr2 = cloud2.Crop(boxxx2);

}



void bdipl(int, void*)
{

    // prvo uslikati dok je još žuto iza, prikazati sve ove 2D crno/bijelo itd i 3D sa žutim iza i onda kad se maknu indices - to mogu u bzuto sve
    // a ovdje ću maknuti iza žuto da lijepo mogu slikati predmet, ovdje mi je bitno 3D prikazati prvo uslikan i 2D da se vidi distance threshold, pa prikazati kako odsječem sa rectangleom i onda maknuti rukavicu
    // ICP ću prikazati korak po korak, stavim ih u žuto i plavo i prikazuje korak po korak i kasnije konačan rez žuto i plavo i još normalno bez pofarbanog

    Eigen::Vector3d look, up, front;
    double zoom;
    look[0] = 0.0272; look[1] = -0.2475;  look[2] = 0.3232;   //  zadnji parametar mi daje središte gore-dolje po ekranu
    up[0] = -0.0694; up[1] = -0.9768; up[2] = 0.2024;
    front[0] = -0.02795; front[1] = -0.35125; front[2] = -0.03795;
    zoom = 0.2212;

    open3d::geometry::AxisAlignedBoundingBox boxxx1, boxxx2, boxxx3;
    boxxx1.min_bound_[0] = -0.17; boxxx1.min_bound_[1] = -0.15; boxxx1.min_bound_[2] = 0.2;
    boxxx1.max_bound_[0] = 0.20; boxxx1.max_bound_[1] = 0.13; boxxx1.max_bound_[2] = 0.5;
    boxxx1.color_[0] = 1; boxxx1.color_[1] = 0; boxxx1.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu1 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx1);

    boxxx2.min_bound_[0] = -0.11; boxxx2.min_bound_[1] = -0.05; boxxx2.min_bound_[2] = 0.15;
    boxxx2.max_bound_[0] = 0.2; boxxx2.max_bound_[1] = 0.135; boxxx2.max_bound_[2] = 0.45;
    boxxx2.color_[0] = 1; boxxx2.color_[1] = 0; boxxx2.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu2 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx2);

    boxxx3.min_bound_[0] = -0.18; boxxx3.min_bound_[1] = -0.05; boxxx3.min_bound_[2] = 0.15;
    boxxx3.max_bound_[0] = 0.13; boxxx3.max_bound_[1] = 0.135; boxxx3.max_bound_[2] = 0.46;
    boxxx3.color_[0] = 1; boxxx3.color_[1] = 0; boxxx3.color_[2] = 0;
    std::shared_ptr<open3d::geometry::AxisAlignedBoundingBox> buu3 = std::make_shared<open3d::geometry::AxisAlignedBoundingBox>(boxxx3);





    rs2::threshold_filter thresholdFilter;
    rs2::points points1, points2, points3;
    string naziv1, naziv2, naziv3;

    // 1.cam : R=110,255 ; G=30,255 ; B=12,36
    // 2.cam : R=100,255 ; G=50,255 ; B=14,36
    // 3.cam : R=110,255 ; G=30,255 ; B=15,36

    Mat hsv_image, frame_threshold, img_bw, imgbw_flat, colorImage1, colorImage2, colorImage3;
    std::vector<int> veki;
    std::vector<long unsigned int> rez_vect1, rez_vect2, rez_vect3;
    //********************************************** Spremanje slika **********************************************
    colorImage1 = imread("2D_cam1_first.png");
    cvtColor(colorImage1, hsv_image, COLOR_BGR2HSV);
    inRange(hsv_image, Scalar(10, 50, 6), Scalar(36, 255, 255), frame_threshold); // inRange(hsv_image,Scalar(15,30,110), Scalar(36,255,255),frame_threshold);
    bitwise_not(frame_threshold, frame_threshold);
    cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
    cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
    bitwise_and(colorImage1, frame_threshold, colorImage1);
//    imwrite("2D_cam1_first.png", colorImage1); // t = 0.01s

    imgbw_flat = img_bw.reshape(1, 1); // flat 1d
    veki = imgbw_flat.row(0);
    for (int j = 0; j < veki.size(); j++)
    {
        if (veki.at(j) == 255)
        {
            veki.at(j) = 1;
            veki.at(j) = veki.at(j) * j;
            rez_vect1.push_back(veki.at(j));
        }
    }


    colorImage2 = imread("2D_cam2_first.png");
    cvtColor(colorImage2, hsv_image, COLOR_BGR2HSV);
    inRange(hsv_image, Scalar(14, 150, 56), Scalar(36, 255, 255), frame_threshold); // inRange(hsv_image,Scalar(12,30,110), Scalar(36,255,255),frame_threshold);
    bitwise_not(frame_threshold, frame_threshold);
    cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
    cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
    bitwise_and(colorImage2, frame_threshold, colorImage2);
//    imwrite("2D_cam2_first.png", colorImage2); // t = 0.01s

    imgbw_flat = img_bw.reshape(1, 1); // flat 1d
    veki = imgbw_flat.row(0);
    for (int j = 0; j < veki.size(); j++)
    {
        if (veki.at(j) == 255)
        {
            veki.at(j) = 1;
            veki.at(j) = veki.at(j) * j;
            rez_vect2.push_back(veki.at(j));
        }
    }


    colorImage3 = imread("2D_cam3_first.png");
    cvtColor(colorImage3, hsv_image, COLOR_BGR2HSV);
    inRange(hsv_image, Scalar(13, 70, 70), Scalar(36, 255, 255), frame_threshold); // inRange(hsv_image,Scalar(12,30,110), Scalar(36,255,255),frame_threshold);
    bitwise_not(frame_threshold, frame_threshold);
    cv::threshold(frame_threshold, img_bw, 128.0, 255.0, THRESH_BINARY);
    cvtColor(frame_threshold, frame_threshold, COLOR_GRAY2BGR);
    bitwise_and(colorImage3, frame_threshold, colorImage3);
//    imwrite("2D_cam3_first.png.png", colorImage3); // t = 0.01s

    imgbw_flat = img_bw.reshape(1, 1); // flat 1d
    veki = imgbw_flat.row(0);          //    fInitializeDevices();
    for (int j = 0; j < veki.size(); j++)
    {
        if (veki.at(j) == 255)
        {
            veki.at(j) = 1;
            veki.at(j) = veki.at(j) * j;
            rez_vect3.push_back(veki.at(j));
        }
    }
    //********************************************** Spremanje slika **********************************************
    //********************************************** Cloudovi *****************************************************

    open3d::io::ReadPointCloud("3D_cam1_first.ply", cloud1);
    open3d::io::ReadPointCloud("3D_cam2_first.ply", cloud2);
    open3d::io::ReadPointCloud("3D_cam3_first.ply", cloud3);


    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1); //   t = 0.00161014 s
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2); //   t = 0.00161014 s
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3); //   t = 0.00161014 s
//    naziv1 = "3D_cam1_first.ply"; //   t = 4.455e-06 s
//    naziv2 = "3D_cam2_first.ply"; //   t = 4.455e-06 s
//    naziv3 = "3D_cam3_first.ply"; //   t = 4.455e-06 s
    open3d::visualization::DrawGeometries({cloudPtr1}, "cloud1", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr2}, "cloud2", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr3}, "cloud3", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "Prvi cloudovi", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3, buu1, buu2, buu3}, "Prvi cloudovi s box", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::io::WritePointCloudToPLY(naziv1, *cloudPtr1, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//    open3d::io::WritePointCloudToPLY(naziv2, *cloudPtr2, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//    open3d::io::WritePointCloudToPLY(naziv3, *cloudPtr3, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample

//    cloudPtr1 = cloud1.SelectByIndex(rez_vect1);
//    cloudPtr2 = cloud2.SelectByIndex(rez_vect2);
//    cloudPtr3 = cloud3.SelectByIndex(rez_vect3);
//    naziv1 = "3D_cam1_inRange.ply"; //   t = 4.455e-06 s
//    naziv2 = "3D_cam2_inRange.ply"; //   t = 4.455e-06 s
//    naziv3 = "3D_cam3_inRange.ply"; //   t = 4.455e-06 s
//    open3d::visualization::DrawGeometries({cloudPtr1}, "select by index 1st", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::visualization::DrawGeometries({cloudPtr2}, "select by index 2nd", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::visualization::DrawGeometries({cloudPtr3}, "select by index 3rd", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "select by index", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
//    open3d::io::WritePointCloudToPLY(naziv1, *cloudPtr1, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//    open3d::io::WritePointCloudToPLY(naziv2, *cloudPtr2, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
//    open3d::io::WritePointCloudToPLY(naziv3, *cloudPtr3, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample

//    open3d::io::ReadPointCloud(naziv1, cloud1);
//    open3d::io::ReadPointCloud(naziv2, cloud2);
//    open3d::io::ReadPointCloud(naziv3, cloud3);
    cloudPtr1 = cloud1.VoxelDownSample(0.002); //  t = 0.00780749 s
    cloudPtr2 = cloud2.VoxelDownSample(0.002); //  t = 0.00780749 s
    cloudPtr3 = cloud3.VoxelDownSample(0.002); //  t = 0.00780749 s
//    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3, buu1, buu2, buu3}, "voxel down", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    cloudPtr1 = cloud1.Crop(boxxx1);
    cloudPtr2 = cloud2.Crop(boxxx2);
    cloudPtr3 = cloud3.Crop(boxxx3);
    naziv1 = "3D_cam1_final.ply"; //   t = 4.455e-06 s
    naziv2 = "3D_cam2_final.ply"; //   t = 4.455e-06 s
    naziv3 = "3D_cam3_final.ply"; //   t = 4.455e-06 s
    open3d::io::WritePointCloudToPLY(naziv1, *cloudPtr1, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    open3d::io::WritePointCloudToPLY(naziv2, *cloudPtr2, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    open3d::io::WritePointCloudToPLY(naziv3, *cloudPtr3, true); //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample
    open3d::visualization::DrawGeometries({cloudPtr1}, "after crop 1", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr2}, "after crop 2", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr3}, "after crop 3", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "after crop&voxel all", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.Clear();cloud2.Clear();cloud3.Clear();
    open3d::io::ReadPointCloud(naziv1, cloud1);
    open3d::io::ReadPointCloud(naziv2, cloud2);
    open3d::io::ReadPointCloud(naziv3, cloud3);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);

    // ICPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP

    //cloud1.PaintUniformColor({1, 0.706, 0});
    //cloud3.PaintUniformColor({0, 0.651, 0.929});

//    open3d::geometry::PointCloud point_cloud1, point_cloud2, point_cloud3;
//    point_cloud1= cloud1 + cloud2 + cloud3;
//    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr4 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All three camera together", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    cloud12 = cloud2;
    cloud13 = cloud3;
    cloud12.Transform(transformMatrix12);
    cloud13.Transform(transformMatrix13);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds transformed", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

    cloud1.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud12.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloud13.EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(0.01 * 2.0, 30));
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud13);
    //    point_cloud1.PaintUniformColor({1, 0.706, 0});
    //    point_cloud2.PaintUniformColor({0, 0.651, 0.929});
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_1.ply", *cloudPtr1, true);
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_2.ply", *cloudPtr2, true);
    //    open3d::io::WritePointCloudToPLY("cloud_nr1_3.ply", *cloudPtr3, true);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds with normals", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);
    //    open3d::io::ReadPointCloud("cloud_nr1_1.ply", cloud1);
    //    open3d::io::ReadPointCloud("cloud_nr1_2.ply", cloud2);
    //    open3d::io::ReadPointCloud("cloud_nr1_3.ply", cloud3);
    //    cloud12 = point_cloud2;  // jer će se cloud12 mjenjati pa da nam cloud2 ostane original    TU NEŠTO NE RADI AHA NEMA TRANSFORMATION MATRIX. PRVI FIND STEREOCALIB ONDA OVO SVE
    //    cloud12.Transform(transformMatrix12);
    //    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(point_cloud1);
    //    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
    //    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2});
    Eigen::Matrix4d transformMat;
    transformMat.setZero(4,4);
    transformMat(0,0) = 1;
    transformMat(1,1) = 1;
    transformMat(2,2) = 1;
    transformMat(3,3) = 1;

    /***   ICP   ***/                      //STAVITI LOOK DA DOBRO VIDI KAK SE PRIMIĆU BLIŽE CLOUDOVI!!!!
    double threshold = 0.005;
    cout << "Kreće ICP:" << endl;
    open3d::pipelines::registration::ICPConvergenceCriteria crit;
    /*criteria (open3d.registration.ICPConvergenceCriteria, optional, default=pipelines::registration::ICPConvergenceCriteria class with
     * relative_fitness=1.000000e-06, relative_rmse=1.000000e-06, and max_iteration=30) – Convergence criteria*/
    crit.max_iteration_=50;
    crit.relative_fitness_=1.000000;
    crit.relative_rmse_=1.000000e-06;

//    crit.max_iteration_=1;
//    cloud1.PaintUniformColor({1, 0.706, 0});
//    cloud13.PaintUniformColor({0, 0.651, 0.929});
//    for(int i=0; i<20; i++){
//        open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud13, cloud1,
//                                                                                                                     threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);
//        cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl;
//        cout << "Fitness 12: " << reg12.fitness_ << endl;
//        cout << "rmse12: " << reg12.inlier_rmse_ << endl << endl;
//        cloud12.Transform(reg12.transformation_);
//        cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
//        cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud12);
//        open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2}, "Iteration nr: " + to_string(i), 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

//    }

    crit.max_iteration_=50;


    open3d::pipelines::registration::RegistrationResult reg12 = open3d::pipelines::registration::RegistrationICP(cloud1, cloud12,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);

    open3d::pipelines::registration::RegistrationResult reg13 = open3d::pipelines::registration::RegistrationICP(cloud13, cloud1,
                                                                                                                 threshold, transformMat, open3d::pipelines::registration::TransformationEstimationPointToPlane(), crit);


    cout << "ICP transformacijska matrica: " << endl << reg12.transformation_ << endl << endl;
    cout << "ICP transformacijska matrica: " << endl << reg13.transformation_ << endl << endl;
    cout << "Fitness 12: " << reg12.fitness_ << endl;
    cout << "rmse12: " << reg12.inlier_rmse_ << endl << endl;
    cout << "Fitness 13: " << reg13.fitness_ << endl;
    cout << "rmse13: " << reg13.inlier_rmse_ << endl << endl;


    open3d::io::ReadPointCloud(naziv1, cloud1);
    open3d::io::ReadPointCloud(naziv2, cloud2);
    open3d::io::ReadPointCloud(naziv3, cloud3);
    cloud2.Transform(transformMatrix12);
    cloud3.Transform(transformMatrix13);
    cloud1.Transform(reg12.transformation_);
    cloud3.Transform(reg13.transformation_);
    cloud3.Transform(reg12.transformation_);
    cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);
    open3d::visualization::DrawGeometries({cloudPtr1, cloudPtr2, cloudPtr3}, "All clouds after ICP", 1500, 1000, 100, 50, false, false, false, &look, &up, &front, &zoom);

}




//----------------------------------- Buttons functions ----------------------------------
//----------------------------------- Functions ----------------------------------
void fInitializeGUI()   // inicijalizacija funkcije OpenCV GUI-ja
{
    result1.release();
    input1.release();

    result2.release();
    input2.release();

    result3.release();
    input3.release();

    destroyAllWindows();   //remove all windows

    //create a gui window:
    namedWindow("Output1");   // Create window
    moveWindow("Output1", 1000, 300);   // move windows into this area of desktop

    namedWindow("Output2");   // Create window
    moveWindow("Output2", 1500, 300);   // move windows into this area of desktop

    namedWindow("Output3");   // Create window
    moveWindow("Output3", 500, 300);   // move windows into this area of desktop

    //-------------------Control panel-------------------
    //Start/load/close
    String nameb1 = "";
    //----------
    //    createButton("Start cameras", bStartCameras, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Get new images", bGetNewImage, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Get new cloud", bGetNewCloud, &nameb1, QT_PUSH_BUTTON, 0);
    //    createButton("Open3D vis", bOpen3dVisualizer, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Cloud Clean", bCloudCleaning, &nameb1, QT_PUSH_BUTTON, 0);
    //    createButton("Single_Cam_Live", bSingleCamLive, &nameb1, QT_PUSH_BUTTON, 0);
    //    createButton("Single_Cam_save", bSingleCamSave, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Transform clouds 1-2", bCloudTransform12, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Transform cloudos 1-3", bCloudTransform13, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Transform clouds 1-2-3", bCloudTransform123, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Live_Stereo1-2", bLiveStereo12, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Live_Stereo1-3", bLiveStereo13, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Live_Stereo123", bLiveStereo123, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("save frames", bsaveframes, &nameb1, QT_PUSH_BUTTON, 0);
    //   createButton("save pics 3s", bsavepics, &nameb1, QT_PUSH_BUTTON, 0);
//    createButton("saveCloud3D", bsavecloud, &nameb1, QT_PUSH_BUTTON, 0);
//    createButton("read Clouds", breadcloud, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("pok_suho", bpoksuho, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("pok_all", bpokall, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("b3T", b3T, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("singlePC", bsingle_cloud_icp, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("cedev", bcede, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Zuto", bzuto, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("bDipl", bdipl, &nameb1, QT_PUSH_BUTTON, 0);
    //   createButton("Stereo_save_pic", bStereoSavePic, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Find_Stereo_Calib", bFindStereoCalibration, &nameb1, QT_PUSH_BUTTON, 0);
    //    createButton("Z_os", bPomakZ, &nameb1, QT_PUSH_BUTTON, 0);
    //    createButton("LIVE STREAM", bCloudLiveStream, &nameb1, QT_PUSH_BUTTON, 0);
    createButton("Shut down", bStopCallback, &nameb1, QT_PUSH_BUTTON, 0);
//    createTrackbar("Low R","Object Detection", &low_r, 255, on_low_r_thresh_trackbar);
    createTrackbar("Low R","", &low_r, 255, on_low_r_thresh_trackbar);
    createTrackbar("High R","", &high_r, 255, on_high_r_thresh_trackbar);
    createTrackbar("Low G","", &low_g, 255, on_low_g_thresh_trackbar);
    createTrackbar("High G","", &high_g, 255, on_high_g_thresh_trackbar);
    createTrackbar("Low B","", &low_b, 255, on_low_b_thresh_trackbar);
    createTrackbar("High B","", &high_b, 255, on_high_b_thresh_trackbar);

//    createTrackbar("Bilateral_kernel", "", &bilateral_kernel, 100, start_callback);
    //-------------------Control panel-------------------
}
void fApplyAlgorithms()
{
    result1.release();
    result1 = input1.clone();
    result2.release();
    result2 = input2.clone();
    result3.release();
    result3 = input3.clone();

    imshow("Output1", result1);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    imshow("Output2", result2);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    imshow("Output3", result3);   //otvara prozor s imenom Output i prikazuje sliku u matrici result
    waitKey(1);   //čekaj tipku

    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr1 = std::make_shared<open3d::geometry::PointCloud>(cloud1);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr2 = std::make_shared<open3d::geometry::PointCloud>(cloud2);
    std::shared_ptr<open3d::geometry::PointCloud> cloudPtr3 = std::make_shared<open3d::geometry::PointCloud>(cloud3);

    //    vis1.CreateVisualizerWindow("Visualizer 1", 960, 540, 0, 0);
    //    vis1.AddGeometry(cloudPtr1);
    //    vis1.UpdateRender();
    //    vis1.Run();
    open3d::visualization::DrawGeometries({cloudPtr1}, "First camera");
    open3d::visualization::DrawGeometries({cloudPtr2}, "Second camera");
    open3d::visualization::DrawGeometries({cloudPtr3}, "Third camera");
}
//void fApplyLiveAlgorithms()
//{
//    if (cloud1->size()>0)
//    {
//        pcl::transformPointCloud(*cloud1, *transformed_cloud2, transform_2);   //   SDK za R i T point clouda
//        viewerM.removePointCloud("sample_cloud_1");
//        viewerM.addPointCloud(transformed_cloud2,  "sample_cloud_1", 0);
//        //viewer3.createViewPort (0.0,0.0,0.5,1.0,0);
//        //viewer3.setBackgroundColor(0,0,0,0); // background color dark
//        //viewer3.addText("sample_cloud_1", 10, 10, "right", 0);
//    }
//    if (cloud2->size()>0)
//    {
//        //********************* apply transformation_2 on cloud 2
//        //        pcl::transformPointCloud(*cloud2, *transformed_cloud2, transform_2);   //   SDK za R i T point clouda
//        //viewer3.addPointCloud(transformed_cloud,  "cloud3", 0);
//        viewerM.removePointCloud("sample_cloud_2");
//        viewerM.addPointCloud(cloud2,  "sample_cloud_2", 0);
//        viewerM.spinOnce(1,1);
//    }
//}
tuple<Mat, Mat, double> fSingleCamCalibration()
{//    open3d::io::WritePointCloudToPLY("zuti_cloud3.ply", *cloudPtr3, true);   //   OVO TRAJE ~0,3s ako je full pointcloud ;;; t = 0.000485747 s ako je downsample

    int idNr = 0;

    Mat img, imgs, gray;
    vector<Point2f> corners;
    vector<vector<Point2f>> imagePoints;
    vector<vector<Point3f>> objectPoints;

    vector<Mat> rvecs, tvecs;
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    double rms;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    for (int i = 0; i < 9; i++)        //ovaj cijela funkcija dalje bi trebala biti u foru dalje DO IMAGE POINTS
    {
        idNr = idNr + 1;
        string idStr1 = "images_saved_cam" + to_string(nrCam) + "/pic" + to_string(idNr) + ".png";

        img.empty();
        img = imread(idStr1);
        cvtColor(img, gray, COLOR_BGR2GRAY);

        bool found1 = false;

        found1 = findChessboardCorners(img, patternSize, corners,
                                       CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (found1)
        {
            cornerSubPix(gray, corners, Size(5, 5), Size(-1, -1),
                         TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray, patternSize, corners, found1);
            //            imshow(idStr1, gray1);

            vector< Point3f > obj;
            for (int i = 0; i < boardHeight; i++)
            {
                for (int j = 0; j < boardWidth; j++)
                {
                    obj.push_back(Point3f((float)j * squareSize, (float)i * squareSize, 0));
                }
            }
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
        else
        {
            cout << "Chessboard corners not found!" << endl << endl;
        }
    }

    if( nrCam == 1)
    {
        rms = calibrateCamera(objectPoints, imagePoints, img.size(), intrinsic,
                              distCoeffs, rvecs, tvecs, CALIB_FIX_K4|CALIB_FIX_K5);
        cout << "Intrinzična od PRVE: " << intrinsic << endl;
        cout << "Distorzijaaaa od PRVE: " << distCoeffs << endl;
        cout << "rms od PRVE Kalibracije iznosi: " << rms << endl << endl << endl;
        cv::FileStorage storage("Camera_1_Parameters.yml", cv::FileStorage::WRITE);
        storage << "intrinsic1" << intrinsic;
        storage << "distCoeffs1" << distCoeffs;
        storage << "rms1" << rms;
        storage.release();
    }
    if( nrCam == 2)
    {
        rms = calibrateCamera(objectPoints, imagePoints, img.size(), intrinsic,
                              distCoeffs, rvecs, tvecs, CALIB_FIX_K4|CALIB_FIX_K5);
        cout << "Intrinzična od DRUGE: " << intrinsic << endl;
        cout << "Distorzijaaaa od DRUGE: " << distCoeffs << endl;
        cout << "rms od DRUGE Kalibracije iznosi: " << rms << endl << endl << endl;
        cv::FileStorage storage("Camera_2_Parameters.yml", cv::FileStorage::WRITE);
        storage << "intrinsic2" << intrinsic;
        storage << "distCoeffs2" << distCoeffs;
        storage << "rms2" << rms;
        storage.release();
    }
    if( nrCam == 3)
    {
        rms = calibrateCamera(objectPoints, imagePoints, img.size(), intrinsic,
                              distCoeffs, rvecs, tvecs, CALIB_FIX_K4|CALIB_FIX_K5);
        cout << "Intrinzična od TRECE: " << intrinsic << endl;
        cout << "Distorzijaaaa od TRECE: " << distCoeffs << endl;
        cout << "rms od TRECE Kalibracije iznosi: " << rms << endl << endl << endl;
        cv::FileStorage storage("Camera_3_Parameters.yml", cv::FileStorage::WRITE);
        storage << "intrinsic3" << intrinsic;
        storage << "distCoeffs3" << distCoeffs;
        storage << "rms3" << rms;
        storage.release();
    }

    nrCam = 0;

    return make_tuple(intrinsic,distCoeffs,rms);
}
void fDrawAxisAndCorners(Mat imgDraw, vector<Point2f> cornersDraw, vector< Point3f > objDraw, string izlazDraw, bool foundDraw)
{
    vector<Point2f> objectPointsPlanar;
    for (size_t i = 0; i < objDraw.size(); i++)
    {
        objectPointsPlanar.push_back(Point2f(objDraw[i].x, objDraw[i].y));
    }

    vector<Point2f> imagePoints;
    undistortPoints(cornersDraw, imagePoints, cameraMatrix1, distortionCoefficients1);

    Mat H = findHomography(objectPointsPlanar, imagePoints);

    double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                       H.at<double>(1,0)*H.at<double>(1,0) +
                       H.at<double>(2,0)*H.at<double>(2,0));
    H /= norm;
    Mat c1  = H.col(0);
    Mat c2  = H.col(1);
    Mat c3 = c1.cross(c2);
    Mat tvec = H.col(2);
    Mat R1(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        R1.at<double>(i,0) = c1.at<double>(i,0);
        R1.at<double>(i,1) = c2.at<double>(i,0);
        R1.at<double>(i,2) = c3.at<double>(i,0);
    }
    //cout << "R (before polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << endl;
    Mat W, U, Vt;
    SVDecomp(R1, W, U, Vt);
    R1 = U*Vt;
    //    cout << "R (after polar decomposition):\n" << R1 << "\ndet(R): " << determinant(R1) << endl;
    Mat rvec;
    Rodrigues(R1, rvec);
    drawFrameAxes(imgDraw, cameraMatrix1, distortionCoefficients1, rvec, tvec, 2*squareSize);

    drawChessboardCorners(imgDraw, patternSize, cornersDraw, foundDraw);
    imshow(izlazDraw, imgDraw);
    waitKey(1);
}

void on_low_r_thresh_trackbar(int, void *)
{
    low_r = min(high_r-1, low_r);
    setTrackbarPos("Low R","", low_r);
}

void on_high_r_thresh_trackbar(int, void *)
{
    high_r = max(high_r, low_r+1);
    setTrackbarPos("High R", "", high_r);
}

void on_low_g_thresh_trackbar(int, void *)
{
    low_g = min(high_g-1, low_g);
    setTrackbarPos("Low G","", low_g);
}

void on_high_g_thresh_trackbar(int, void *)
{
    high_g = max(high_g, low_g+1);
    setTrackbarPos("High G", "", high_g);
}

void on_low_b_thresh_trackbar(int, void *)
{
    low_b= min(high_b-1, low_b);
    setTrackbarPos("Low B","", low_b);
}

void on_high_b_thresh_trackbar(int, void *)
{
    high_b = max(high_b, low_b+1);
    setTrackbarPos("High B", "", high_b);
}


//----------------------------------- Functions ----------------------------------
/************************************************ MAIN ************************************************/
int main()
{

    fInitializeDevices();
    fInitializeGUI();

    srand((unsigned int)time(NULL));//random generator for nameing different files/objects

    while (waitKey(33) != 27)
    {

    }
    return 1;   // 1 jer je to oznaka za gresku. nasa main funkcija ima beskonacnu while petlju pa ne bi smjela nikada vratiti ovu jedinicu
}
/************************************************ MAIN ************************************************/


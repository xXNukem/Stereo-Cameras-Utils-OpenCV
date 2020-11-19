#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.01905f; //meters
const float arucoSquareDimension = 0.1016f; //meters
const Size chessboardDimension = Size(6,9);

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f> & corners)
{
    for(int i=0;i<boardSize.height;i++)
    {
        for(int j=0; j<boardSize.width; j++)
        {
            corners.push_back(Point3f(j*squareEdgeLength, i*squareEdgeLength,0.0f));
        }
    }
}

void getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults=false)
{
    for(vector<Mat>::iterator iter = images.begin();iter != images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        bool found= findChessboardCorners(*iter, Size(9,6),pointBuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        
        if(found)
        {
            allFoundCorners.push_back(pointBuf);
        }

        if(showResults)
        {
            drawChessboardCorners(*iter, Size(9,6), pointBuf,found);
            imshow("Looking for corners", *iter);
            waitKey(0);
        }
    }
}

static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

void cameraCalibration(string filename, vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCore)
{
    vector<vector<Point2f>> checkerboardImageSpacePoints;
    vector<vector<Point3f>> worldSpaceCornerPoints(1);
    vector<Mat> rVectors, tVectors;
    Mat distanceCoefficients = Mat::zeros(8,1,CV_64F);

    getChessboardCorners(calibrationImages,checkerboardImageSpacePoints,false);
    createKnownBoardPosition(boardSize,squareEdgeLength,worldSpaceCornerPoints[0]);
    worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(),worldSpaceCornerPoints[0]);
    calibrateCamera(worldSpaceCornerPoints,checkerboardImageSpacePoints,boardSize,cameraMatrix,distanceCoefficients,rVectors,tVectors);

      saveCameraParams(filename, Size(640,480),Size(6,9), 0.1016f, 0,0, cameraMatrix, distanceCoefficients,rVectors,tVectors,vector<float>(),vector<vector<Point2f> >(),0 );

}



int main(int argv, char** argc)
{
    Mat frame;
    Mat drawToFrameLeft;
    Mat drawToFrameRight;

    Mat cameraMatrix = Mat::eye(3,3, CV_64F);

    Mat distanceCoefficients;

    vector<Mat> savedImagesLeft;
    vector<Mat> savedImagesRight;

    vector<vector<Point2f>> markerCorners, rejectedCandidates;

    VideoCapture vid(0);

    if(!vid.isOpened())
    {
        return 0;
    }

    int framesPerSecond=20;

    namedWindow("WebCam1", CV_WINDOW_AUTOSIZE);
    namedWindow("WebCam2", CV_WINDOW_AUTOSIZE);

    while(true)
    {
        if(!vid.read(frame))
        {
            break;
        }

        Mat fullFrame = frame (Rect(0,0,1280,480));
        Mat leftFrame = frame (Rect(0,0,640,480));
        Mat rightFrame = frame (Rect(640,0,640,480));

        vector<Vec2f> foundPointsLeft;
        vector<Vec2f> foundPointsRight;
        bool foundLeft= false;
        bool foundRight= false;

        foundLeft=findChessboardCorners(leftFrame, chessboardDimension,foundPointsLeft, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

        leftFrame.copyTo(drawToFrameLeft);

        drawChessboardCorners(drawToFrameLeft,chessboardDimension,foundPointsLeft,foundLeft);


        foundRight=findChessboardCorners(rightFrame, chessboardDimension,foundPointsRight, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FAST_CHECK);

        rightFrame.copyTo(drawToFrameRight);

        drawChessboardCorners(drawToFrameRight,chessboardDimension,foundPointsRight,foundRight);

        if(foundLeft == true && foundRight == true)
        {
            imshow("WebCam1",drawToFrameLeft);
            imshow("WebCam2",drawToFrameRight);
        }
        else
        {
            imshow("WebCam1", leftFrame);
            imshow("WebCam2",rightFrame);
        }

        char character = waitKey(1000/framesPerSecond);

        switch (character)
        {
        case ' ':
            //saving image SPACE KEY
            if(foundLeft == true || foundRight == true)
            {
                Mat tempLeft;
                Mat tempRight;
                
                leftFrame.copyTo(tempLeft);
                rightFrame.copyTo(tempRight);

                savedImagesLeft.push_back(tempLeft);
                savedImagesRight.push_back(tempRight);

                cout<<"Image saved to buffer\n";
            }
            break;

        case 'c':
        //start calibration  PRESS ENTER
        if(savedImagesLeft.size()>15 && savedImagesRight.size()>15)
        {

            cameraCalibration("left_params.yml",savedImagesLeft, chessboardDimension, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
            cameraCalibration("right_params.yml",savedImagesRight, chessboardDimension, calibrationSquareDimension, cameraMatrix, distanceCoefficients);

            cout<<"Calibration file saved";
        }
        else
        {
            cout << "Not enought images \n";
        }
        
        
        break;

        case 'e':
        //exit PRES ESCAPE
        break;
        
        default:
            break;
        }
        
    }

    return 0;
}
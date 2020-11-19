#include <iostream>
#include <exception>
#include <string>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/videoio/videoio.hpp>
using namespace std;
using namespace cv;

const cv::String keys =
"{help h usage ? |      | Frame Splitter for Stereo cameras}"
  "{@video  |<none>| input video}"
  ;

int main(int argc, char* const* argv)
{
    int retCode = EXIT_SUCCESS;
    try{
    cv::CommandLineParser parser(argc, argv, keys);
    parser.about("Application name v1.0.0");
    if (parser.has("help"))
    {
      parser.printMessage();
      return 0;
    }
    String video = parser.get<String>(0);

    Mat frame;
    VideoCapture inputVideo;
    inputVideo.open(video);
    VideoWriter leftCamera("outLeft.avi",CV_FOURCC('M','J','P','G'),24, Size(640,480));
    VideoWriter rightCamera("outRight.avi",CV_FOURCC('M','J','P','G'),24, Size(640,480));

    if(!inputVideo.isOpened())
    {
        return 0;
    }

    cout<<"Splitting... Wait a moment! \n";

    while (true)
    {
        if(!inputVideo.read(frame))
        {
            break;
        }

        Mat fullFrame = frame (Rect(0,0,1280,480));
        Mat leftFrame = frame (Rect(0,0,640,480));
        Mat rightFrame = frame (Rect(640,0,640,480));

        leftCamera.write(leftFrame);
        rightCamera.write(rightFrame);


    }
    inputVideo.release();
    leftCamera.release();
    rightCamera.release();

    cout<<"Split completed \n"<<endl;

    }catch (std::exception& e)
  {
    std::cerr << "CAPTURED EXCEPTION!!! " << e.what() << std::endl;
    retCode = EXIT_FAILURE;
  }


return retCode;
}
# OpenCV scripts for stereo cameras

This repo contains some scripts that may come handy if you are starting to work with OpenCV. Here you will find some utils to work with stereo cameras.

# calibrator_stereo

This script allows you to calibrate an stereo camera with only one USB port. 

## Usage

Just execute the script and press space to take some pictures of your chessboard when the points appear. When you have at least 15 of them, press c and the .yml file will be generated.


# frame_splitter
If you are working with stereo cameras, you may want to separate both lefth and right frames into different videos. With this script you can do it easily.

## Usage

You only need to have your video ready in some directory. So add it as a first argument in the comand line and thats it.
Notice that the code is set to work on 1280x480 resolutions.

Harris-Corner-Detector
======================

Implementation of the Harris Corner Detection Algorithm with maxima suppression.
For I/O and displaying the transformed image, OpenCV is used, whereas the Harris Corner Detector is completely self-implemented.

Prerequisite software
----------
- OpenCV [http://opencv.org/]
- CMake [http://www.cmake.org/install/]

Build instructions
----------
- open a shell and cd into a directory wished to git-clone in.
- clone the repo: 
> git clone https://github.com/alexanderb14/Harris-Corner-Detector.git
- cd into it: 
> cd Harris-Corner-Detector
- create a build directory and cd into it: 
> mkdir build && cd build
- run cmake and initiate the build process: 
> cmake .. && make

Usage
----------
./Ex2 [jpg-file]
- sample image files can be found in "sampleData".

/*
 *      Author: alexanderb
 */

#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "Harris.h"

using namespace cv;
using namespace std;

//Harris algorithm parameters
// Specifies the sensitivity factor of the Harris algorithm (0 < k < 0.25)
float k = 0.25;
// Size of the box filter that is applied to the integral images
int boxFilterSize = 3;
// dimension of the maxima suppression box around a maxima
int maximaSuppressionDimension = 10;

//UI parameters
// dimension of the objects showing a maxima in the image
int markDimension = 5;
// constant for the slider-value division
float divisionConstant = 1000000;

//Global variables
int slider_valueMaxsp = 10;
int slider_valueMeanfilter = 2;
int slider_valuePercentage = 1;
float percentage;
Mat m_img;

void doHarris() {
    // compute harris
    Harris harris(m_img, k, boxFilterSize);

    // get vector of points wanted
    vector<pointData> resPts = harris.getMaximaPoints(percentage, boxFilterSize, maximaSuppressionDimension);
    // cout << resPts.size() << " Points" << endl;

    Mat _img = Util::MarkInImage(m_img, resPts, markDimension);
    imshow("HarrisCornerDetector", _img);
}

//-----------------------------------------------------------------------------------------------
void CallbackTrackbarMaxsp(int value, void* userdata) {
    maximaSuppressionDimension = slider_valueMaxsp;
    cout << "maximaSuppressionDimension:" << maximaSuppressionDimension << endl;

    doHarris();
}

//-----------------------------------------------------------------------------------------------
void CallbackTrackbarMeanfilter(int value, void* userdata) {
    boxFilterSize = slider_valueMeanfilter;

    cout << "boxFilterSize:" << boxFilterSize << endl;

    doHarris();
}

//-----------------------------------------------------------------------------------------------
void CallbackTrackbarPercentage(int value, void* userdata) {
    // calculate percentage of the points with top-harris response wished to display
    percentage = slider_valuePercentage / (float) divisionConstant;

    cout << "percentage:" << percentage << endl;

    doHarris();
}

//-----------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    // read image from file + error handling
    Mat img;

    if (argc == 1) {
        cout << "No image provided! Usage: ./Ex1 [path to image]" << endl << "Using default image: haus.jpg" << endl;

        img = imread("haus.jpg");
    } else {
        img = imread(argv[1]);
    }

    if(img.rows > 100 || img.cols > 100) {
        int newrows = 600;
        int newcols = img.cols * newrows / img.rows;

        resize(img, img, Size(newcols, newrows), 0, 0, INTER_CUBIC);
    }
    img.copyTo(m_img);

    // create UI and show the image
    namedWindow("HarrisCornerDetector", 1);

    createTrackbar("maxima suppresison", "HarrisCornerDetector", &maximaSuppressionDimension, 14, CallbackTrackbarMeanfilter);
    createTrackbar("size mean filter", "HarrisCornerDetector", &slider_valueMeanfilter, 20, CallbackTrackbarMeanfilter);
    createTrackbar("number points", "HarrisCornerDetector", &slider_valuePercentage, 1000, CallbackTrackbarPercentage);

    imshow("HarrisCornerDetector", img);
    waitKey(0);

    return 0;

}

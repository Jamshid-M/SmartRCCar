
#include <opencv2/opencv.hpp>
#include <vector>

#ifndef ADVANCED_IMAGE_PROC_LANEDETECTOR_H
#define ADVANCED_IMAGE_PROC_LANEDETECTOR_H

using namespace cv;

class LaneDetector {

private:
    double img_center;
    double img_size;
    bool right_flag;
    bool left_flag;
    double right_m;
    Point2i right_b;
    double left_m;
    Point2i left_b;


public:
    Mat deNoise(Mat frame);
    Mat yellowRange(Mat frame, int hue_min, int sat_min,
                    int bri_min, int hue_max, int sat_max, int bri_max);
    Mat edgeDetection(Mat frame, int value);
    Mat mask(Mat frame);
    std::vector<cv::Vec4i> houghLines(Mat frame);
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, Mat img_edges);
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);
    double predictTurn();
    int plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);
    double polyFit(std::vector<double> x, std::vector<double> y, int degree, int size, double eq_num);
    int getLineState();
};


#endif //ADVANCED_IMAGE_PROC_LANEDETECTOR_H

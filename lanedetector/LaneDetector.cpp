//
// Created by Jamshid Mamatkulov on 2019-04-26.
//

#include "LaneDetector.h"
#include <iostream>
#include <vector>


Mat LaneDetector::deNoise(Mat frame) {
    Mat output;
    GaussianBlur(frame, output, Size(3, 3), 0, 0);
    return output;
}

Mat LaneDetector::yellowRange(Mat frame, int hue_min, int sat_min,
                              int bri_min, int hue_max, int sat_max, int bri_max) {
    Mat output;
    cvtColor(frame, output, CV_BGR2HSV);
    inRange(output, Scalar(hue_min, sat_min, bri_min), Scalar(hue_max, sat_max, bri_max), output);
    return output;
}

Mat LaneDetector::edgeDetection(Mat frame, int value) {
    Mat output;
    cvtColor(frame, output, COLOR_BGR2GRAY);
    Canny(output, output, value, value*3);
    return output;
}

Mat LaneDetector::mask(Mat frame) {

    Mat mask = cv::Mat::zeros(frame.size(), frame.type());
    Mat output;

    Point pts[6] = {
            cv::Point(0, frame.rows),
            cv::Point(frame.cols*0.05, frame.rows*0.8),
            cv::Point(frame.cols*0.4, frame.rows*0.7),
            cv::Point(frame.cols*0.6, frame.rows*0.7),
            cv::Point(frame.cols*0.95, frame.rows*0.8),
            cv::Point(frame.cols, frame.rows)
    };

    fillConvexPoly(mask, pts, 6, cv::Scalar(255));
    bitwise_and(frame, mask, output);
    return output;
}

std::vector<cv::Vec4i> LaneDetector::houghLines(Mat frame){
    std::vector<cv::Vec4i> line;

    HoughLinesP(frame, line, 1, CV_PI/180, 20, 20, 30);
    return line;
}

std::vector<std::vector<cv::Vec4i> > LaneDetector::lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges) {
    std::vector<std::vector<cv::Vec4i> > output(2);
    size_t j = 0;
    cv::Point ini;
    cv::Point fini;
    double slope_thresh = 0.3;
    std::vector<double> slopes;
    std::vector<cv::Vec4i> selected_lines;
    std::vector<cv::Vec4i> right_lines, left_lines;

    // Calculate the slope of all the detected lines
    for (auto i : lines) {
        ini = cv::Point(i[0], i[1]);
        fini = cv::Point(i[2], i[3]);

        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        double slope = (static_cast<double>(fini.y) - static_cast<double>(ini.y))/(static_cast<double>(fini.x) - static_cast<double>(ini.x) + 0.00001);

        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (std::abs(slope) > slope_thresh) {
            slopes.push_back(slope);
            selected_lines.push_back(i);
        }
    }

    // Split the lines into right and left lines
    img_center = static_cast<double>((img_edges.cols / 2));
    img_size = static_cast<double>(img_edges.rows);
    left_flag = false;
    right_flag = false;
    while (j < selected_lines.size()) {
        if (slopes[j] > 0) {
            right_flag = true;
        } else if (slopes[j] < 0 ) {
            left_flag = true;
        }
        j++;
    }
    j = 0;
    while (j < selected_lines.size()) {
        ini = cv::Point(selected_lines[j][0], selected_lines[j][1]);
        fini = cv::Point(selected_lines[j][2], selected_lines[j][3]);
        // Condition to classify line as left side or right side
        if (slopes[j] > 0 && ((fini.x > img_center && ini.x > img_center) || !left_flag)) {
            right_lines.push_back(selected_lines[j]);
        } else if (slopes[j] < 0 && ((fini.x < img_center && ini.x < img_center) || !right_flag)) {
            left_lines.push_back(selected_lines[j]);
        }

        j++;
    }
    output[0] = right_lines;
    output[1] = left_lines;

    return output;
}

std::vector<cv::Point> LaneDetector::regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage) {
    std::vector<cv::Point> output(4);
    cv::Point ini;
    cv::Point fini;
    cv::Point ini2;
    cv::Point fini2;
    cv::Vec4d right_line;
    cv::Vec4d left_line;
    std::vector<cv::Point> right_pts;
    std::vector<cv::Point> left_pts;

    // If right lines are being detected, fit a line using all the init and final points of the lines
    if (right_flag) {
        for (auto i : left_right_lines[0]) {
            ini = cv::Point(i[0], i[1]);
            fini = cv::Point(i[2], i[3]);

            right_pts.push_back(ini);
            right_pts.push_back(fini);
        }

        if (right_pts.size() > 0) {
            // The right line is formed here
            cv::fitLine(right_pts, right_line, cv::DIST_L2, 0, 0.01, 0.01);
            right_m = right_line[1] / right_line[0];
            right_b = cv::Point(right_line[2], right_line[3]);
        }
    }else{
        right_m = inputImage.cols;
        right_b = cv::Point(inputImage.cols, inputImage.rows);
    }

    // If left lines are being detected, fit a line using all the init and final points of the lines
    if (left_flag) {
        for (auto j : left_right_lines[1]) {
            ini2 = cv::Point(j[0], j[1]);
            fini2 = cv::Point(j[2], j[3]);

            left_pts.push_back(ini2);
            left_pts.push_back(fini2);
        }

        if (left_pts.size() > 0) {
            // The left line is formed here
            cv::fitLine(left_pts, left_line, cv::DIST_L2, 0, 0.01, 0.01);
            left_m = left_line[1] / left_line[0];
            left_b = cv::Point(left_line[2], left_line[3]);
        }
    }else{
        left_m = inputImage.cols;
        left_b = cv::Point(0, inputImage.rows);
    }

    // One the slope and offset points have been obtained, apply the line equation to obtain the line points
    int ini_y = inputImage.rows;
    int fin_y = inputImage.rows/3;

    double right_ini_x = ((ini_y - right_b.y) / right_m) + right_b.x;
    double right_fin_x = ((fin_y - right_b.y) / right_m) + right_b.x;

    double left_ini_x = ((ini_y - left_b.y) / left_m) + left_b.x;
    double left_fin_x = ((fin_y - left_b.y) / left_m) + left_b.x;

    output[0] = cv::Point(right_ini_x, ini_y);
    output[1] = cv::Point(right_fin_x, fin_y);
    output[2] = cv::Point(left_ini_x, ini_y);
    output[3] = cv::Point(left_fin_x, fin_y);

    return output;
}

double LaneDetector::predictTurn() {
    std::string output;
    double vanish_x;
    double thr_vp = 50;

//    vanish_x = static_cast<double>(((right_m*right_b.x) - (left_m*left_b.x) - right_b.y + left_b.y) / (right_m - left_m));

    double vanish_left_x, vanish_right_x;

    vanish_left_x = (left_m*left_b.x - left_b.y)/left_m;
    vanish_right_x = (right_m*right_b.x - right_b.y)/right_m;

    vanish_x = (vanish_left_x + vanish_right_x)/2;

    return vanish_x;
}

int LaneDetector::plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn) {
    std::vector<cv::Point> poly_points;
    cv::Mat output;

    // Create the transparent polygon for a better visualization of the lane
    inputImage.copyTo(output);
    poly_points.push_back(lane[2]);
    poly_points.push_back(lane[0]);
    poly_points.push_back(lane[1]);
    poly_points.push_back(lane[3]);
    cv::fillConvexPoly(output, poly_points, cv::Scalar(0, 0, 255), cv::LINE_AA, 0);
    cv::addWeighted(output, 0.3, inputImage, 1.0 - 0.3, 0, inputImage);

    // Plot both lines of the lane boundary
    cv::line(inputImage, lane[0], lane[1], cv::Scalar(0, 255, 255), 5, cv::LINE_AA);
    cv::line(inputImage, lane[2], lane[3], cv::Scalar(0, 255, 255), 5, cv::LINE_AA);

    // Plot the turn message
    cv::putText(inputImage, turn, cv::Point(50, 90), cv::FONT_HERSHEY_COMPLEX_SMALL, 3, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);

//     Show the final output image
    cv::namedWindow("Lane", cv::WINDOW_NORMAL);

    cv::imshow("Lane", inputImage);
    return 0;
}

int LaneDetector::getLineState(){

    if(right_flag && left_flag)
        return 2;
    if(right_flag && !left_flag)
        return 1;
    if(!right_flag && left_flag)
        return 0;
    if (!right_flag && !left_flag)
        return -1;
}

double LaneDetector::polyFit(std::vector<double> x, std::vector<double> y, int degree, int size, double eq_num) {

    int i,j,k;
    std::cout.precision(4);                        //set precision
    std::cout.setf(std::ios::fixed);

    double X[2*degree+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*degree+1;i++)
    {
        X[i]=0;
        for (j=0;j<size;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    double B[degree+1][degree+2],a[degree+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=degree;i++)
        for (j=0;j<=degree;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    double Y[degree+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<degree+1;i++)
    {
        Y[i]=0;
        for (j=0;j<size;j++)
            Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i=0;i<=degree;i++)
        B[i][degree+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    degree=degree+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations

    for (i=0;i<degree;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<degree;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=degree;j++)
                {
                    double temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }

    for (i=0;i<degree-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<degree;k++)
        {
            double t=B[k][i]/B[i][i];
            for (j=0;j<=degree;j++)
                B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
    for (i=degree-1;i>=0;i--)                //back-substitution
    {                        //x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][degree];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<degree;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }

    double sum = 0;
    for (i=0;i<degree;i++){
        sum += a[i] * pow(eq_num, i);
    }
    return sum;
}
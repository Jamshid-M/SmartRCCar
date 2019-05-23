#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <sstream>
#include "lanedetector/LaneDetector.h"
#include "ultrasonic/Ultrasonic.h"
#include <raspicam/raspicam_cv.h>
#include <softPwm.h>
#include <wiringPi.h>
#include <csignal>
#include <pthread.h>

#define LEFT_MOTOR_FORWARD 1
#define LEFT_MOTOR_REVERSE 4
#define RIGHT_MOTOR_FORWARD 5
#define RIGHT_MOTOR_REVERSE 6
#define MAX_SPEED 100
#define MIN_SPEED 0

#define LEFT_IR_PIN 10
#define RIGHT_IR_PIN 11
#define RIGHT_IR 26
#define LEFT_IR 27
#define TRIG_PIN 28
#define ECHO_PIN 29

using namespace cv;
using namespace std;

int getConfigValue(string);
void *ir_tracer(void*);
void *ultrasonic(void*);

void initDCMotor();
void go(int leftSpeed, int rightSpeed);
void stop();
void signalHandler( int signum );
void *traffic_light(void*);
void getStaticArea(Mat frame);

bool ir_tracer_on = false;
bool ultrasonic_on = false;
bool isRed = false;
bool cameraHasStarted = false;
bool isStaticMode = false;
bool pedestrian = false;
bool isStop = false;
bool isStopReached = false;
bool isTurnDetected = false;

int lineState = 0;
int start_time;

Ultrasonic sonic;
LaneDetector ld;
Mat global_frame;
CascadeClassifier cascade, cascade_left;

int main() {
    
    Mat frame, output, traffic_frame;
    Mat canny;
    string turn;
    
    signal(SIGINT, signalHandler);
    //    frame = imread("/Users/jamshid/CLionProjects/untitled/img2.jpg", 1 );
    //    VideoCapture capture("/Users/jamshid/Downloads/outcpp 2.avi");
    
    raspicam::RaspiCam_Cv capture;
    capture.open();
    
    if(wiringPiSetup() == -1)
        return 0;
    
    initDCMotor();
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    
    pthread_t ultra_thread, traffic_thread, ir_thread;
    pthread_create(&ultra_thread, nullptr, ultrasonic, nullptr);
    pthread_create(&traffic_thread, nullptr, traffic_light, nullptr);
    pthread_create(&ir_thread, nullptr, ir_tracer, nullptr);
    
    int w = 320, h = 240;
    start_time = micros();
    
    while (true){
        //        if(!capture.read(frame))
        //            break;
        
        capture.grab();
        capture.retrieve(frame);
        
        cameraHasStarted = true;
        
        resize(frame, frame, Size(320, 240));
        frame.copyTo(output);
        frame.copyTo(canny);
        frame.copyTo(global_frame);
        frame.copyTo(traffic_frame);
        
        
        //        imshow("static", red_hue_image);
        
        //        getStaticArea(traffic_frame);
        frame = ld.deNoise(frame);
        canny = ld.edgeDetection(frame, getConfigValue("canny_value"));
        frame = ld.yellowRange(frame, getConfigValue("hue_min"), getConfigValue("sat_min"),
                               getConfigValue("bri_min"), getConfigValue("hue_max"), getConfigValue("sat_max"),
                               getConfigValue("bri_max"));
        bitwise_and(frame, canny, frame);
        dilate(frame,frame,Mat(), Point(-1,-1), 3);
        
        frame = ld.mask(frame);
        
        vector<cv::Vec4i> lines = ld.houghLines(frame);
        
        vector<vector<Vec4i> > output1 = ld.lineSeparation(lines, frame);
        
        vector<cv::Point> left_right_lines;
        
        double vanish_x = ld.predictTurn();
        string turn = "";
        
        double percent = 0;
        double img_center = frame.cols/2;
        
        lineState = ld.getLineState();
        
        percent = abs(img_center - vanish_x)/img_center*100;
        
        //        cout<<percent<<endl;
        if(vanish_x < (img_center - 50)&&!ir_tracer_on&&!ultrasonic_on&&!isRed&&!isStop&&!isTurnDetected){
            if(percent > 300){
                turn = "Left 300";
                go(getConfigValue("negative_spin"), getConfigValue("spin"));
            }
            else if(percent > 250){
                turn = "Left 250";
                go(getConfigValue("smooth_extra_hard"), getConfigValue("max"));
            }
            else if(percent > 200){
                turn = "Left 200";
                go(getConfigValue("smooth_hard"), getConfigValue("max"));
            }
            else if(percent > 150){
                turn = "Left 150";
                go(getConfigValue("smooth_medium"), getConfigValue("max"));
            }
            else if(percent > 100){
                turn = "Left 100";
                go(getConfigValue("smooth"), getConfigValue("max"));
            }
            else if(percent >= 0){
                turn = "Light left";
                go(getConfigValue("smooth_light"), getConfigValue("max"));
            }
        } else if(vanish_x > (img_center + 50)&&!ir_tracer_on&&!ultrasonic_on&&!isRed&&!isStop&&!isTurnDetected){
            if(percent > 300){
                turn = "Right 300";
                go(getConfigValue("spin"), getConfigValue("negative_spin"));
            }
            else if(percent > 250){
                turn = "Right 250";
                go(getConfigValue("max"), getConfigValue("smooth_extra_hard"));
            }
            else if(percent > 200){
                turn = "Right 200";
                go(getConfigValue("max"), getConfigValue("smooth_hard"));
            }
            else if(percent > 150){
                turn = "Right 150";
                go(getConfigValue("max"), getConfigValue("smooth_medium"));
            }
            else if(percent > 100){
                turn = "Right 100";
                go(getConfigValue("max"), getConfigValue("smooth"));
            }
            else if(percent >= 0){
                turn = "Light Right";
                go(getConfigValue("max"), getConfigValue("smooth_light"));
            }
            
        } else if(vanish_x >= (img_center - 50) && vanish_x <= (img_center + 50)&&!ir_tracer_on&&!ultrasonic_on&&!isRed&&!isStop&&!isTurnDetected){
            turn = "Straight";
            go(getConfigValue("max"), getConfigValue("max"));
        }
        
        //        cout<<turn<<endl;
        ld.plotLane(output, left_right_lines = ld.regression(output1, frame), turn);
        ld.plotLane(output, left_right_lines = ld.regression(output1, frame), turn);
        
        if(cvWaitKey(1) == 'q'){
            stop();
            break;
        }
        
    }
    
    return 0;
}

int getConfigValue(string key){
    ifstream infile("config.txt");
    
    string k;
    int value;
    while (infile >> k >> value){
        if(k == key)
            return value;
    }
    
    return 0;
}

void initDCMotor(){
    pinMode(LEFT_MOTOR_FORWARD, SOFT_PWM_OUTPUT);
    pinMode(LEFT_MOTOR_REVERSE, SOFT_PWM_OUTPUT);
    pinMode(RIGHT_MOTOR_FORWARD, SOFT_PWM_OUTPUT);
    pinMode(RIGHT_MOTOR_REVERSE, SOFT_PWM_OUTPUT);
    
    softPwmCreate(LEFT_MOTOR_FORWARD, MIN_SPEED, getConfigValue("max"));
    softPwmCreate(LEFT_MOTOR_REVERSE, MIN_SPEED, getConfigValue("max"));
    softPwmCreate(RIGHT_MOTOR_FORWARD, MIN_SPEED, getConfigValue("max"));
    softPwmCreate(RIGHT_MOTOR_REVERSE, MIN_SPEED, getConfigValue("max"));
}


void go(int leftSpeed, int rightSpeed){
    if ((abs(leftSpeed)>100) || (abs(rightSpeed)>100))
        return;
    
    if (leftSpeed>0) {
        softPwmWrite (LEFT_MOTOR_REVERSE,0);
        softPwmWrite (LEFT_MOTOR_FORWARD,leftSpeed);
    }
    else {
        softPwmWrite (LEFT_MOTOR_FORWARD,0);
        softPwmWrite (LEFT_MOTOR_REVERSE,-1*leftSpeed);
    }
    
    if (rightSpeed>0) {
        softPwmWrite (RIGHT_MOTOR_REVERSE,0);
        softPwmWrite (RIGHT_MOTOR_FORWARD,rightSpeed);
    }
    else {
        softPwmWrite (RIGHT_MOTOR_FORWARD,0);
        softPwmWrite (RIGHT_MOTOR_REVERSE,-1*rightSpeed);
    }
}

//- DC motor stop------------------------------------------------------------------------------------------------------------------------
void stop() {
    
    softPwmWrite(LEFT_MOTOR_FORWARD, MIN_SPEED);
    softPwmWrite(LEFT_MOTOR_REVERSE, MIN_SPEED);
    softPwmWrite(RIGHT_MOTOR_FORWARD, MIN_SPEED);
    softPwmWrite(RIGHT_MOTOR_REVERSE, MIN_SPEED);
}

// tracking line with IR_TRACER
void *ir_tracer(void*){
    
    int nLValue =0, nRValue=0;
    
    pinMode(LEFT_IR_PIN, INPUT);
    pinMode(RIGHT_IR_PIN, INPUT);
    
    while(1) {
        nLValue = digitalRead(LEFT_IR_PIN);
        nRValue = digitalRead(RIGHT_IR_PIN);
        
        
        
        if(nLValue == 0 && nRValue==1 && !isStaticMode && !isStop) {
            ir_tracer_on = true;
            go(getConfigValue("ir_speed"), getConfigValue("negative_ir_speed"));
            continue;
        }
        else if(nLValue == 1&&nRValue==0 && !isStaticMode && !isStop){
            ir_tracer_on = true;
            go(getConfigValue("negative_ir_speed"), getConfigValue("ir_speed"));
            continue;
        }
        
        ir_tracer_on = false;
        //        else if(nLValue == 0&&nRValue==0)
        //            go(getConfigValue("max"), getConfigValue("max"));
        //            stop();
        
    }
}

// checks if there is an obstacle
void *ultrasonic(void*){
    
    sonic.init();
    int static_counter = 0;
    bool isCenter = true;
    bool calculated = false;
    int current = -1;
    int diff = 0;
    int current_line = 0;
    
    pinMode(LEFT_IR, INPUT);
    pinMode(RIGHT_IR, INPUT);
    
    while(1){
        double distance = sonic.getDistance();
        int nRValue = 0;
        int nLValue = 0;
        
        if(distance > 0 && distance < getConfigValue("distance") && !isStop) {
            ultrasonic_on = true;
            
            //cout<<micros() - start_time<<endl;
            
             nLValue = digitalRead(LEFT_IR_PIN);
             nRValue = digitalRead(RIGHT_IR_PIN);
            
             if(current != -1){
                 diff = micros() - current;
             }
            
             current = micros();
            //prevent checking the same object twice by ultrasonic
             if(nLValue == 1 || nRValue == 1){
                 if(diff > 1000000){
                     static_counter++;
                     cout<<"FOUND"<<diff<<endl;
                 }
             }

            
            cout<<static_counter<<endl;
            
            if(static_counter>0){
                stop();
                isStaticMode = true;
                
                if(current_line%2!=0){
                    go(-1 * getConfigValue("static_right"), -1 * getConfigValue("static_right"));
                    delay(getConfigValue("back_delay"));
                    go(getConfigValue("static_right"), getConfigValue("static_left"));
                    delay(getConfigValue("turn_delay"));
                    current_line++;
                } else{
                    go(-1 * getConfigValue("static_right"), -1 * getConfigValue("static_right"));
                    delay(getConfigValue("back_delay"));
                    go(getConfigValue("static_left"), getConfigValue("static_right"));
                    delay(getConfigValue("turn_delay"));
                    current_line++;
                }
            }
            else
                stop();
            
        }
        else {
            calculated = false;
            ultrasonic_on = false;
        }
    }
}

// working with traffic signs and traffic light
void *traffic_light(void* ){
    
    delay(1000);
    Rect roi(0, 0, 320, 120);
    Mat stop_frame;
    cascade.load("cascades/Stop.xml");
    cascade_left.load("cascades/left.xml");
    
    while (1){
        if(cameraHasStarted){
            Mat red_mat;
            Mat blue_frame;
            global_frame.copyTo(red_mat);
            
            Mat cut = red_mat(roi);
            cut.copyTo(stop_frame);
            Mat hsv;
            
            cvtColor(cut, hsv, CV_BGR2HSV);
            hsv.copyTo(blue_frame);
            
            Mat redLowerMask, redHigherMask;
            inRange(hsv, Scalar(0, 150, 100), Scalar(10, 255, 255), redLowerMask);
            inRange(hsv, Scalar(170, 150, 100), Scalar(179, 255, 255), redHigherMask);
            
            Mat red_hue_image = redLowerMask|redHigherMask;
            GaussianBlur(red_hue_image, red_hue_image, Size(5, 5), 2, 2);
            
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            vector<cv::Point> approx;
            
            int counter = 0;
  
            
            findContours( red_hue_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            if(contours.size()>0){
                
                vector <Rect> st;
                
                cascade.detectMultiScale(stop_frame, st, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(5, 5));
                
                if(st.size()>0 && !isStopReached){
                    cout<<"Stop sign"<<endl;
                    isStop = true;
                    stop();
                    delay(3000);
                    isStop = false;
                    isRed = false;
                    isStopReached = true;
                }
                else {
                    for (int i = 0; i < contours.size(); i++) {
                        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02,
                                         true);
                        double area = contourArea(contours[i]);
                        
                        if(area > 300 && area < 1100){
                            isRed = true;
                            stop();
                            break;
                        } else {
                            isRed = false;
                        }
                    }
                }
            } else
                isRed = false;
            
            Mat blueMask;
            inRange(blue_frame, Scalar(88, 40, 50), Scalar(126, 255, 255), blueMask);
            GaussianBlur(blueMask, blueMask, Size(5, 5), 2, 2);
            
            vector<vector<Point> > blue_contours;
            vector<Vec4i> blue_hierarchy;
            
            findContours( blueMask, blue_contours, blue_hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            
            vector <Rect> lft;
            for (int i = 0; i < blue_contours.size(); i++){
                double area = contourArea(blue_contours[i]);
                if(area<1000 && isStaticMode){
                    
                    cascade_left.detectMultiScale(stop_frame, lft, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(5, 5));
                    if(lft.size()>0) {
                        cout<<"Left"<<endl;
                        isTurnDetected = true;
                        go(getConfigValue("static_left2"), getConfigValue("static_right"));
                        delay(getConfigValue("delay3"));
                        stop();
                        exit(1);
                    }
                }
            }
            if(blue_contours.size()==0)
                isTurnDetected = false;
            delay(100);
        }
    }
}

void getStaticArea(Mat frame){
    Mat canny;
    frame.copyTo(canny);
    
    cvtColor(canny, canny, CV_BGR2GRAY);
    Canny(canny, canny, 100, 300);
    
    imshow("canny", canny);
    cvtColor(frame, frame, CV_BGR2HSV);
    inRange(frame, Scalar(0, 0, 120), Scalar(255, 135, 255), frame);
    imshow("white", frame);
    frame = ld.mask(frame);
    
    //    bitwise_and(canny, frame, frame);
    //    Canny(frame, frame, 100, 200);
    
    imshow("static", frame);
    
    vector<vector<Point> > contours;
    vector<cv::Point> approx;
    
    cv::findContours(frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    
    int counter = 0;
    for (int i = 0; i < contours.size(); ++i) {
        
        cout<<contourArea(contours[i])<<endl;
    }
    
}

void signalHandler( int signum ) {
    cout << "Interrupt signal (" << signum << ") received.\n";
    stop();
    exit(1);
}


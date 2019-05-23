# SmartRCCar
## **Self Driving car with Computer Vision using OpenCV library**
Create executable file with command  <br />
**g++ main.cpp LaneDetector.cpp Ultrasonic.cpp -o main -lwiringPi -pthread -I/usr/local/include -L/usr/local/lib -L/opt/vc/lib -lraspicam -lraspicam_cv -lmmal -lmmal_core -lmmal_util `pkg-config --cflags --libs opencv`**

![](https://drive.google.com/uc?export=view&id=1ZBU3ZykYDZgPd88-yEJz9ojcN139LQX-)

## **Step 1**
Taking frame as input from RaspiCam

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/original.png)

## **Step 2**
Denoising frame with GaussianBlur, OpenCV fucntion

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/blurred.png)

## **Step 3**
Finding edges with Canny edge detection, OpenCV function

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/Canny.png)

## **Step 4**
Finding yellow colors in the frame with inRange OpenCV function

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/inRange.png)

## **Step 5**
After applying step 3 and step 4 we use bitwise_and for two frames, this function returns intersection points of two frames. <br/>
Also applying dilate function will help you with finding lines by increasing pixels near our line pixels.

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/Canny+inRange.png)

## **Step 6**
Finally we have to mask our frame for avoiding unnecessary lines and pixels in our frame 

![alt text](https://github.com/Jamshid-M/SmartRCCar/blob/master/examples/mask.png)

After this step we just apply HoughLines function and find right and left lines appropriately.<br/>
With the help of right and left lines we will predict turn in predictTurn function

In this project I've made also traffic light recognition with color filtering and traffic sign recognition with Cascade classifier (xml)

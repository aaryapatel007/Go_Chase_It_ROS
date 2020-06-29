#include <stdio.h>
#include <iostream>
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvaux.h>
#include <math.h>
#include <cxcore.h>
#include <highgui.h>

static const char WINDOW[] = "Image window";

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

ros::ServiceClient client;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish(): it_(n_)
  {
      //Topic you want to subscribe

     // sub1 = n_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndPublish::process_image_callback, this);
      img_sub = it_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndPublish::hough_process_image_callback, this);
  }

  void drive_robot(float lin_x, float ang_z){

    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if(!client.call(srv))
      ROS_ERROR("failed to call service command_robot");

  }


  void hough_process_image_callback(const sensor_msgs::ImageConstPtr& img){
  	cv_bridge::CvImagePtr cv_ptr;
   try
    {
      cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    Mat src_gray;

    cvtColor(cv_ptr->image, src_gray, CV_BGR2GRAY);

    GaussianBlur( src_gray, src_gray, Size(9, 9), 0, 0);

    vector<Vec3f> circles;

	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/6, 60, 30, 5, 70 );

    ROS_INFO("Number of circles = %d", (int)circles.size());



    float div_image = src_gray.cols / 3;   //divides the image width in 3 regions(left, center and right)

    float lin_x = 0.0, ang_z = 0.0;
    int ball_detected = 0;

    if(circles.size() == 0){
      //stop the robot if ball is not detected
      drive_robot(lin_x, ang_z);
      ROS_INFO_STREAM("ball not detected! halting the robot");
      return;
    }

    //Take the first circle
    float x = (float)circles[0][0];
    float y = (float)circles[0][1];

    if(x <= div_image){
      lin_x = 0.0;   			//drive the robot left
      ang_z = 0.2;
    }
    else if(x >= 2 * div_image){
      lin_x = 0.0;			  	//drive the robot right
      ang_z = -0.2;
    }
    else{
      lin_x = 0.3;			   //drive the robot forward
      ang_z = 0.0;
    }

    // drive the robot to the ball
    drive_robot(lin_x, ang_z);
    ROS_INFO_STREAM("ball detected! moving the robot towards the ball");

  }

private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub;
  ros::Subscriber sub1;

};//End of class SubscribeAndPublish

int main(int argc, char ** argv){

    ros::init(argc, argv, "process_image");

    ros::NodeHandle n;

    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}

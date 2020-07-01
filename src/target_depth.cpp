#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <iostream>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <person_navigation/Polar.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>   
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/utility.hpp>

#define PI 3.14159265

//camera intrinsic parameters
#define Cx 322.4541015625
#define Cy 243.06459045410156
#define Fx 616.3174438476562
#define Fy 616.46801757812

ros::Publisher pub_depth_angle;

sensor_msgs::Image depth_image;
sensor_msgs::CompressedImage colour_image_latest;
sensor_msgs::CompressedImage colour_image_sync;

void bbox_callback(const geometry_msgs::PointConstPtr& msgcenter)
{
	//obtain pixel coordinate
	float x = msgcenter->x;
	float y = msgcenter->y;

}

void bbox_callback(const geometry_msgs::PointConstPtr& msgcenter)
{
	//obtain pixel coordinate
	float x = msgcenter->x;
	float y = msgcenter->y;

	//Z depth calculation using depth image
	cv::Mat DepthImageCopy_;
	cv_bridge::CvImageConstPtr cam_depth;
	cam_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
	DepthImageCopy_ = cam_depth->image.clone();
	float depth_val = (float)DepthImageCopy_.at<float>( 240, 320 );
	float Z = depth_val;

	//X and Y coordinate calculation/Z depth calculation using depth image
	cv::Mat DepthImageCopy_;
	cv_bridge::CvImageConstPtr cam_depth;
	cam_depth = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_32FC1);
	DepthImageCopy_ = cam_depth->image.clone();
	float depth_val = (float)DepthImageCopy_.at<float>( 240, 320 );
	float Z = depth_val;

	//X and Y coordinate calculation
	float X = ((320-Cx)*Z)/Fx;           //X=((U-Cx)*Z)/fx
    float Y = ((240-Cy)*Z)/Fy;           //Y=((V-Cy)*Z)/fy

	std::cout << "Z: " << depth_val << "   X: " << X << "   Y: " << Y << std::endl;

	//angle calculation
	float angle = atan((-X / 1000.0) / (Z / 1000.0)) * (180/PI);	
	std::cout<<angle<<std::endl;

	//publish depth and angle
	person_navigation::Polar pub_msg;
    pub_msg.angle = angle;
  	pub_msg.depth = Z;
   	pub_depth_angle.publish(pub_msg);
}

//store most current depth image
void depth_callback(const sensor_msgs::Image& msgdepth)
{
	if
		depth_image = msgdepth;
		colour_image_sync = colour_image_latest;
}

void colour_callback(const sensor_msgs::CompressedImage& msgcolour)
{
	
	colour_image_latest = msgcolour;
}

int main (int argc, char **argv)
{
	//ROS node setup
	ros::init(argc, argv, "target_depth");
	ros::NodeHandle nh;
	ros::Subscriber sub_depthImage = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_callback);
	ros::Subscriber sub_colourImage = nh.subscribe("/camera/color/image_raw/compressedrosto", 1, colour_callback);
	ros::Subscriber sub_boundingBox = nh.subscribe("/person_tracking/bbox_center", 1, bbox_callback);

	pub_depth_angle = nh.advertise<person_navigation::Polar>("/person_navigation/depth_angle", 10);

	ros::spin();
}
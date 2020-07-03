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
#include <person_navigation/update_tracker.h>

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
ros::ServiceClient client;

sensor_msgs::Image depth_image_latest;
sensor_msgs::CompressedImage colour_image_latest;
sensor_msgs::CompressedImage colour_image_sync;

person_navigation::Polar compute_depth_angle(const sensor_msgs::Image& depth_img, const geometry_msgs::Point bbox_center)
{
	//bbox center coordinate
	float x = bbox_center.x;
	float y = bbox_center.y;

	//Z depth calculation using depth image
	cv::Mat DepthImageCopy_;
	cv_bridge::CvImageConstPtr cam_depth;
	cam_depth = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::TYPE_32FC1);
	DepthImageCopy_ = cam_depth->image.clone();
	float depth_val = (float)DepthImageCopy_.at<float>( y, x );
	float Z = depth_val;

	//X and Y coordinate calculation
	float X = ((x-Cx)*Z)/Fx;           //X=((U-Cx)*Z)/fx
    float Y = ((y-Cy)*Z)/Fy;           //Y=((V-Cy)*Z)/fy

	std::cout << "Z: " << depth_val << "   X: " << X << "   Y: " << Y;

	//angle calculation
	float angle = atan((-X / 1000.0) / (Z / 1000.0)) * (180/PI);	
	std::cout<<"   Angle: "<<angle<<std::endl;

	if (std::isnan(angle)){
		angle = 0.0;
	}

	//return depth and angle
	person_navigation::Polar computed_polar;
	computed_polar.depth = Z/1000.0;
	computed_polar.angle = angle;
	return computed_polar;
}

//void bbox_callback(const geometry_msgs::PointConstPtr& msgcenter){}

//store most current depth image
void depth_callback(const sensor_msgs::Image& msgdepth)
{
	depth_image_latest = msgdepth;
}

void colour_callback(const sensor_msgs::CompressedImage& msgcolour)
{
	//sync depth image with color image
	sensor_msgs::Image depth_image_sync = depth_image_latest;
	//std::cout<<depth_image_latest<<std::endl;
	//call service to receive bbox center coordinate
	person_navigation::update_tracker msgsrv;
	msgsrv.request.img = msgcolour;
	try 
	{
		if (client.call(msgsrv))
		{
			//calculate depth and angle
			person_navigation::Polar pub_msg = compute_depth_angle(depth_image_sync, msgsrv.response.coordinates);

			//publish depth and angle
			pub_depth_angle.publish(pub_msg);
		}
		else
		{
			ROS_ERROR("Failed to obtain center coordinate");
		}
	}
	catch (const std::exception& e)
	{
		std::cout<<e.what()<<std::endl;
	}

}


int main (int argc, char **argv)
{
	//ROS node setup
	ros::init(argc, argv, "target_depth");
	ros::NodeHandle nh;
	ros::Subscriber sub_depthImage = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 1, depth_callback);
	ros::Subscriber sub_colourImage = nh.subscribe("/camera/color/image_raw/compressed", 1, colour_callback);
	//ros::Subscriber sub_boundingBox = nh.subscribe("/person_tracking/bbox_center", 1, bbox_callback);

	pub_depth_angle = nh.advertise<person_navigation::Polar>("/person_navigation/depth_angle", 10);

	client = nh.serviceClient<person_navigation::update_tracker>("/person_tracking/update_tracker");

	ros::spin();
}
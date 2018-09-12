#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

ros::Publisher pub;

void callback(std_msgs::String img_file)
{
	Mat img;
	img=imread(img_file.data, CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, img).toImageMsg();
	pub.publish(img_msg);
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"pub_img");
	ros::NodeHandle nh;
	ros::Subscriber sub=nh.subscribe("img_file",2,callback);
	pub=nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",2,false);
	ros::spin();
	return 0;
}
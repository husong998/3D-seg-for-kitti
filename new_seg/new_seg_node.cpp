#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/impl/point_types.hpp>

#include <opencv2/opencv.hpp>

#include <queue>
#include <cmath>

#define THRE 0.5

typedef struct
{
	int s=0,e=0;
	int l;
} run;

using namespace cv;

class new_seg
{
public:
	new_seg();
private:
	ros::NodeHandle nh_;
	ros::Subscriber no_ground_sub_;
	ros::Publisher range_img_pub_;
	void callback_(sensor_msgs::PointCloud2 in_cloud_msg);
	void gen_range_img(float);
	run FindNextRun(int);
	void prov_label(run);
	bool if_merge(run,run);
	void merge(run,run);
	void update_labels();
	void run_seg();
	void visualization();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	boost::shared_ptr<pcl::RangeImageSpherical> range_image_ptr;

	int labelMap[108*1440]={0};
	int rtable[10000]={0};
	int next[10000]={0};
	int tail[10000]={0};
	float *range_array;
};

new_seg::new_seg():nh_("~")
{
	no_ground_sub_ = nh_.subscribe("/points_no_ground", 2, &new_seg::callback_, this);
	range_img_pub_ = nh_.advertise<sensor_msgs::Image>("/range_img", 2, true);
	cloud=pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	range_image_ptr=boost::shared_ptr<pcl::RangeImageSpherical>(new pcl::RangeImageSpherical);
}

void new_seg::gen_range_img(float angularResolution)
{
	float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
	float maxAngleHeight    = (float) (90.0f * (M_PI/180.0f));  // 180.0 degree in radians
	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
	float noiseLevel=0.00;
	float minRange = 0.0f;
	int borderSize = 0;
	range_image_ptr->createFromPointCloud(*cloud, angularResolution, maxAngleWidth, maxAngleHeight,
    	sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	range_array=range_image_ptr->getRangesArray();
}

run new_seg::FindNextRun(int start)
{
	run crun;
	int i;
	static int label=1;
	for (i=start;i<range_image_ptr->height*range_image_ptr->width;i++)
	{
		int c_row=i/range_image_ptr->width, c_col=i%range_image_ptr->width;
		if(fabs(*(range_array+c_row*range_image_ptr->width+c_col)>0.5))
			break;
	}
	crun.s=i;
	for (i=i+1;i<range_image_ptr->height*range_image_ptr->width;i++)
	{
		int c_row=i/range_image_ptr->width, c_col=i%range_image_ptr->width;
		int p_row=(i-1)/range_image_ptr->width, p_col=(i-1)%range_image_ptr->width;
		if(fabs(*(range_array+c_row*range_image_ptr->width+c_col)-*(range_array+p_row*range_image_ptr->width+p_col))>THRE)
			break;
	}
	crun.e=i;
	crun.l=label++;
	return crun;
}

void new_seg::prov_label(run c_run)
{
	int label=c_run.l;
	for (int i=c_run.s;i<c_run.e;i++)
		labelMap[i]=label;
	rtable[label]=label;
	tail[label]=label;
	next[label]=-1;
	label++;
}

bool new_seg::if_merge(run a, run b)
{
	if(rtable[a.l]==rtable[b.l]) return false;
	int i;
	float avg1,avg2;
	avg1=avg2=0;
	for (i=a.s;i<a.e;i++)
		avg1+=*(range_array+i);
	avg1/=a.e-a.s;
	for (i=b.s;i<b.e;i++)
		avg2+=*(range_array+i);
	avg2/=b.e-b.s;
	if(fabs(avg1-avg2)<1.0)
		return true;
	return false;
}

void new_seg::merge(run a, run b)
{	
	if(if_merge(a,b))
	{
		if(rtable[a.l]>rtable[b.l])
			merge(b,a);
		else
		{
			int i=rtable[b.l];
			next[tail[rtable[a.l]]]=rtable[b.l];
			tail[rtable[a.l]]=tail[rtable[b.l]];
			while(i!=-1)
			{
				rtable[i]=rtable[a.l];
				i=next[i];
			}
		}
	}
}

void new_seg::update_labels()
{
	for(int i=0;i<range_image_ptr->width*range_image_ptr->height;i++)
		labelMap[i]=rtable[labelMap[i]];
}

void new_seg::run_seg()
{
	int N=range_image_ptr->width;
	run currentRun;
	std::queue<run> bufferRuns2,bufferRuns1;
	currentRun=FindNextRun(0);
	bufferRuns2.push(currentRun);
	prov_label(currentRun);
	while(currentRun.e<N*range_image_ptr->height)
	{
		currentRun=FindNextRun(currentRun.e);
		bufferRuns2.push(currentRun);
		prov_label(currentRun);
		while(!bufferRuns1.empty() && bufferRuns1.front().e<currentRun.s-2*N) bufferRuns1.pop();
		while(bufferRuns2.front().e<currentRun.s-N)
		{
			bufferRuns1.push(bufferRuns2.front());
			bufferRuns2.pop();
		}
		printf("%d %d\n", currentRun.s, currentRun.e);
		// if(!bufferRuns1.empty() && bufferRuns1.front().s<currentRun.s-2*N)
		// {
		while(!bufferRuns1.empty() && bufferRuns1.front().s<=currentRun.e-2*N)
		{
			merge(bufferRuns1.front(),currentRun);
			bufferRuns1.pop();
		}
		// }
		// if(bufferRuns2.front().s<currentRun.s-N)
		// {
		// 	merge(bufferRuns2.front(),currentRun);
		while(bufferRuns2.front().s<=currentRun.e-N)
		{
			merge(bufferRuns2.front(),currentRun);
			bufferRuns1.push(bufferRuns2.front());
			bufferRuns2.pop();
		}
		// }
	}
	update_labels();
}

void new_seg::visualization()
{
    namedWindow( "Range Image", WINDOW_NORMAL );
    Mat L_color = cv::Mat::zeros(range_image_ptr->height,range_image_ptr->width, CV_8UC3);
    for (int i=0; i<range_image_ptr->height; i++)
    	for (int j=0; j<range_image_ptr->width; j++)
    	{
    		Vec3b& color=L_color.at<Vec3b>(i,j);
    		color[0]=labelMap[i*range_image_ptr->width+j]*100%256;
    		color[1]=labelMap[i*range_image_ptr->width+j]*50%256;
    		color[2]=labelMap[i*range_image_ptr->width+j]*25%256;
    	}
    imshow("Range Image", L_color);
    waitKey(0);
}

void new_seg::callback_(sensor_msgs::PointCloud2 in_cloud_msg)
{
	pcl::fromROSMsg(in_cloud_msg, *cloud);
	float angularResolution = (float) (  0.25f * (M_PI/180.0f));
	gen_range_img(angularResolution);
	run_seg();
	visualization();
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"seg");
	new_seg a;
	ros::spin();
	return 0;
}
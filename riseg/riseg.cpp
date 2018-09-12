#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>

#include <queue>

#define VPoint velodyne_pointcloud::PointXYZIR

constexpr long double operator"" _deg ( long double deg )
{
      return deg*3.141592/180;
}

using namespace std;
using namespace cv;

class riseg
{
public:
	riseg();

private:
	ros::NodeHandle nh_;
	ros::Subscriber no_ground_sub_;
	ros::Publisher range_img_pub_;
	ros::Publisher L_pub_;
  ros::Publisher bsf_pub_;
	cv::Mat gen_range_img_(pcl::PointCloud<VPoint>::Ptr cloud, int r, int c);
	void callback_(sensor_msgs::PointCloud2 in_cloud_msg);
	int find_nearest_(vector<float>& angles, float angle);
	Mat BFS_();

  	vector<float> row_angles_;
  	vector<float> col_angles_;
  	Mat range_img_ = cv::Mat::zeros(64,870, CV_32F); 
  	Mat L_ = cv::Mat::zeros(64,870, CV_16UC1); 
  	int label;
};

int riseg::find_nearest_(vector<float>& angles, float angle)
{
  int found=0;
  found=angles.rend()-upper_bound(angles.rbegin(),angles.rend(),angle);
  if (found==0)
    return found;
  if (found==angles.size())
    return found-1;
  float diff_next = fabs(angles[found]-angle),
        diff_prev = fabs(angle-angles[found-1]);
  return diff_next < diff_prev ? found : found-1;
}

cv::Mat riseg::gen_range_img_(pcl::PointCloud<VPoint>::Ptr cloud, int r, int c)
{
	int i;
	cv::Mat range_img = cv::Mat::zeros(r,c, CV_32F);
  //search thru all the 3d points
  for(i=0;i<cloud->points.size();i++)
  {
    float dist=cloud->points[i].x*cloud->points[i].x+
      cloud->points[i].y*cloud->points[i].y+
      cloud->points[i].z*cloud->points[i].z;
    dist=sqrt(dist);
    if (dist < 0.01) continue;
    //get the position of the pixel
    float row_angle = asin(cloud->points[i].z/dist),
          col_angle = atan2(cloud->points[i].y,cloud->points[i].x);
    int row=find_nearest_(row_angles_,row_angle),
        col=find_nearest_(col_angles_,col_angle);
    if (dist > range_img.at<float>(row,col))
      range_img.at<float>(row,col)=dist;
  }
  return range_img;
}

riseg::riseg():nh_("~")
{
	no_ground_sub_ = nh_.subscribe("/points_no_ground", 2, &riseg::callback_, this);
	range_img_pub_ = nh_.advertise<sensor_msgs::Image>("/range_img", 2, true);
	L_pub_ = nh_.advertise<sensor_msgs::Image>("/range_img_seg", 2, true);
  bsf_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/bfs_run",2,true);
	float rad=180.0_deg,
  lower_angles[32]={-8.47_deg,-8.89_deg,-9.35_deg,-9.74_deg,-10.2_deg,-10.8_deg,-11.29_deg,-11.71_deg,
    -12.16_deg,-12.59_deg,-13.11_deg,-13.63_deg,-14.20_deg,-14.63_deg,-15.13_deg,-15.48_deg,
    -16.09_deg,-16.61_deg,-17.17_deg,-17.62_deg,-18.13_deg,-18.53_deg,-18.99_deg,-19.52_deg,
    -20.03_deg,-20.63_deg,-21.11_deg,-21.51_deg,-21.96_deg,-22.60_deg,-23.12_deg,-12.60_deg};
	int i;
	for (i = 0; i < 870; i++)
	{
		col_angles_.push_back(rad);
		rad-=0.41379_deg;
	}

	rad = 2.63_deg;
	for (i = 0; i < 32; i++)
	{
		row_angles_.push_back(rad);
		rad-=0.3375_deg;
	}
	for (i = 0; i < 32; i++)
		row_angles_.push_back(lower_angles[i]);
    // namedWindow( "Range Image", WINDOW_NORMAL );
}

Mat riseg::BFS_()
{
  	Mat L = cv::Mat::zeros(64,870, CV_16UC1); 
  	bool visited[64][870]={0};
  	label=1;
  	int x,y;
  	float d1,d2;
  	for (int i=0; i < 64; i++)
  		for (int j=0; j < 870; j++)
  		{
  			if(L.at<unsigned short>(i,j)!=0 || range_img_.at<float>(i,j)==0 ) continue;
  			queue<int> qx;
  			queue<int> qy;
  			qx.push(i);
  			qy.push(j);
  			while(!qx.empty())
  			{
  				x=qx.front(); y=qy.front();
  				L.at<unsigned short>(x,y)=label;
  				visited[x][y]=1;
          float phi;
					for(int iy=-1;iy<=1;iy++)
					{
						if(iy+y<0 || iy+y>869 || iy==0 || visited[x][iy+y])
							continue;
						d1=max(range_img_.at<float>(x,y+iy), range_img_.at<float>(x,y));
						d2=min(range_img_.at<float>(x,y+iy), range_img_.at<float>(x,y));
            phi=fabs(col_angles_[y]-col_angles_[y+iy]);
						if( atan2(d2*sin(phi),(d1-d2*cos(phi))) > 10.0_deg)
						{
							qx.push(x);
							qy.push(y+iy);
							visited[x][iy+y]=1;
						}
					}
          for(int ix=-1;ix<=1;ix++)
          {
            if(ix+x<0 || ix+x>63 || ix==0 || visited[x+ix][y])
              continue;
            d1=max(range_img_.at<float>(x+ix,y), range_img_.at<float>(x,y));
            d2=min(range_img_.at<float>(x+ix,y), range_img_.at<float>(x,y));
            phi=fabs(row_angles_[x]-row_angles_[x+ix]);
            if( atan2(d2*sin(phi),(d1-d2*cos(phi))) > 10.0_deg)
            {
              qx.push(x+ix);
              qy.push(y);
              visited[ix+x][y]=1;
            }
          }
  				qx.pop();
  				qy.pop();
  			}
  			label++;
  		}
  	return L;
}

void riseg::callback_(sensor_msgs::PointCloud2 in_cloud_msg)
{
    pcl::PointCloud<VPoint>::Ptr laserCloudIn (new pcl::PointCloud<VPoint>);
    pcl::fromROSMsg(in_cloud_msg, *laserCloudIn);
    // range_img_=gen_range_img_(laserCloudIn);
    int i;

    L_=BFS_();

    // double range_min, range_max;
    // minMaxIdx(range_img_, &range_min, &range_max);
    // convertScaleAbs(range_img_, range_img_, 255/range_max);
    // cvtColor(range_img_, range_img_, COLOR_GRAY2RGB);

    // imshow("Range Image", range_img_);
    // waitKey(0);

    sensor_msgs::ImagePtr ri_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", range_img_).toImageMsg();
    // cout<<range_img_<<endl;
    Mat L_color = cv::Mat::zeros(64,870, CV_8UC3);
    range_img_pub_.publish(ri_msg);
    // int l;
    for (i=0; i<63; i++)
    	for (int j=0; j<870; j++)
    	{
    		Vec3b& color=L_color.at<Vec3b>(i,j);
    		color[0]=L_.at<unsigned short>(i,j)*100%256;
    		color[1]=L_.at<unsigned short>(i,j)*50%256;
    		color[2]=L_.at<unsigned short>(i,j)*25%256;
    	}
    sensor_msgs::ImagePtr L_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", L_color).toImageMsg();
    L_pub_.publish(L_msg);
    cout<<--label<<" segments"<<endl;

    for (i=0;i<laserCloudIn->size();i++)
    {
      float dist=laserCloudIn->points[i].x*laserCloudIn->points[i].x+
        laserCloudIn->points[i].y*laserCloudIn->points[i].y+
        laserCloudIn->points[i].z*laserCloudIn->points[i].z;
      dist=sqrt(dist);
      if (dist < 0.01) continue;
      float row_angle = asin(laserCloudIn->points[i].z/dist),
            col_angle = atan2(laserCloudIn->points[i].y,laserCloudIn->points[i].x);
      int row=find_nearest_(row_angles_,row_angle),
          col=find_nearest_(col_angles_,col_angle);
      laserCloudIn->points[i].intensity=L_.at<unsigned short>(row,col);
    }
    sensor_msgs::PointCloud2 bfs_msg;
    pcl::toROSMsg(*laserCloudIn, bfs_msg);
    bsf_pub_.publish(bfs_msg);
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"RangeImg");
	riseg ri_proc;
	ros::spin();
	return 0;
}
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>

#define VPoint pcl::PointXYZRGBA

constexpr long double operator"" _deg ( long double deg )
{
      return deg*3.141592/180;
}

using namespace std;
using namespace cv;

class riseg
{
public:
	riseg(pcl::PointCloud<VPoint> cloud);
  	int label;

private:
	cv::Mat gen_range_img_(pcl::PointCloud<VPoint> cloud);
	int find_nearest_(vector<float>& angles, float angle);
	Mat BFS_();

  	vector<float> row_angles_;
  	vector<float> col_angles_;
  	Mat range_img_ = cv::Mat::zeros(64,870, CV_32F); 
  	Mat L_ = cv::Mat::zeros(64,870, CV_16UC1); 
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

cv::Mat riseg::gen_range_img_(pcl::PointCloud<VPoint> cloud)
{
	int i;
	cv::Mat range_img = cv::Mat::zeros(64,870, CV_32F);
  //search thru all the 3d points
  for(i=0;i<cloud.points.size();i++)
  {
    float dist=cloud.points[i].x*cloud.points[i].x+
      cloud.points[i].y*cloud.points[i].y+
      cloud.points[i].z*cloud.points[i].z;
    dist=sqrt(dist);
    if (dist < 0.01) continue;
    //get the position of the pixel
    float row_angle = asin(cloud.points[i].z/dist),
          col_angle = atan2(cloud.points[i].y,cloud.points[i].x);
    int row=find_nearest_(row_angles_,row_angle),
        col=find_nearest_(col_angles_,col_angle);
    if (dist < range_img.at<float>(row,col) || 
    	range_img.at<float>(row,col) == 0)
      range_img.at<float>(row,col)=dist;
  }
  return range_img;
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
					for(int iy=-2;iy<=2;iy++)
					{
						if(iy+y<0 || iy+y>869 || iy==0 || visited[x][iy+y])
							continue;
						d1=max(range_img_.at<float>(x,y+iy), range_img_.at<float>(x,y));
						d2=min(range_img_.at<float>(x,y+iy), range_img_.at<float>(x,y));
            phi=fabs(col_angles_[y]-col_angles_[y+iy]);
						if( atan2(d2*sin(phi),(d1-d2*cos(phi))) > 12.0_deg)
						{
							qx.push(x);
							qy.push(y+iy);
							visited[x][iy+y]=1;
						}
					}
          for(int ix=-2;ix<=2;ix++)
          {
            if(ix+x<0 || ix+x>63 || ix==0 || visited[x+ix][y])
              continue;
            d1=max(range_img_.at<float>(x+ix,y), range_img_.at<float>(x,y));
            d2=min(range_img_.at<float>(x+ix,y), range_img_.at<float>(x,y));
            phi=fabs(row_angles_[x]-row_angles_[x+ix]);
            if( atan2(d2*sin(phi),(d1-d2*cos(phi))) > 12.0_deg)
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

riseg::riseg(pcl::PointCloud<VPoint> cloud)
{
	float rad=180.0_deg;
	int i;
	for (i = 0; i < 870; i++)
	{
		col_angles_.push_back(rad);
		rad-=0.41379_deg;
	}

	rad = 2.0_deg;
	for (i = 0; i < 32; i++)
	{
		row_angles_.push_back(rad);
		rad-=0.328125_deg;
	}
	rad = -8.87_deg;
	for (i = 0; i < 32; i++)
	{
		row_angles_.push_back(rad);
		rad-=0.5_deg;
	}
    // namedWindow( "Range Image", WINDOW_NORMAL );
    range_img_=gen_range_img_(cloud);

    L_=BFS_();

    // double range_min, range_max;
    // minMaxIdx(range_img_, &range_min, &range_max);
    // convertScaleAbs(range_img_, range_img_, 255/range_max);
    // cvtColor(range_img_, range_img_, COLOR_GRAY2RGB);

    // imshow("Range Image", range_img_);
    // waitKey(0);

    // sensor_msgs::ImagePtr ri_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", range_img_).toImageMsg();
    // cout<<range_img_<<endl;
    // Mat L_color = cv::Mat::zeros(64,870, CV_8UC3);
    // range_img_pub_.publish(ri_msg);
    // int l;
    // for (int i=0; i<63; i++)
    // 	for (int j=0; j<870; j++)
    // 	{
    // 		Vec3b& color=L_color.at<Vec3b>(i,j);
    // 		color[0]=L_.at<unsigned short>(i,j)*100%256;
    // 		color[1]=L_.at<unsigned short>(i,j)*50%256;
    // 		color[2]=L_.at<unsigned short>(i,j)*25%256;
    // 	}
    // sensor_msgs::ImagePtr L_msg = cv_bridge::CvImage(std_msgs::Header(), "8UC3", L_color).toImageMsg();
}

class filter_bbox
{
public:
	filter_bbox();

private:
	void save_pcd_(sensor_msgs::PointCloud2 cloud_in_msg);
	void save_bbox_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg);
	void save_bbox1_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg);
	void filter_small_boxes_();
	int enclose_(int i,
	vector<int>& inside_boxes);
	void remove_(
	vector<int>& inside_boxes);
	void callback_();
	int get_segs(jsk_recognition_msgs::BoundingBox);
	void select_(int &i);

	pcl::PointCloud<VPoint> cloud;
	ros::NodeHandle nh_;
	ros::Subscriber bboxes_sub_;
	ros::Subscriber bboxes_sub1_;
	ros::Subscriber cloud_sub_;
	ros::Publisher bboxes_pub_;
	jsk_recognition_msgs::BoundingBoxArray bboxes;
	jsk_recognition_msgs::BoundingBoxArray bboxes1;
	jsk_recognition_msgs::BoundingBoxArray result;
	bool bboxes1_flag=false,bboxes_flag=false;
};

filter_bbox::filter_bbox():nh_("~")
{
	cloud_sub_=nh_.subscribe("/points_no_ground",2,&filter_bbox::save_pcd_,this);
	bboxes_sub_=nh_.subscribe("/bboxes",2,&filter_bbox::save_bbox_,this);
	bboxes_sub1_=nh_.subscribe("/bboxes1",2,&filter_bbox::save_bbox1_,this);
	bboxes_pub_=nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bboxes_filtered",2,true);
 	result.header.frame_id = "/velodyne";
}

void filter_bbox::filter_small_boxes_()
{
	bboxes.boxes.insert( bboxes.boxes.end(), bboxes1.boxes.begin(), bboxes1.boxes.end() );
	bboxes1.boxes.clear();
	for (int i = 0; i < bboxes.boxes.size(); i++)
	{
		pcl::PointCloud<VPoint> filtered;

		pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
		float
		minX=bboxes.boxes[i].pose.position.x-bboxes.boxes[i].dimensions.x/2,
		minY=bboxes.boxes[i].pose.position.y-bboxes.boxes[i].dimensions.y/2,
		minZ=bboxes.boxes[i].pose.position.z-bboxes.boxes[i].dimensions.z/2,
		maxX=bboxes.boxes[i].pose.position.x+bboxes.boxes[i].dimensions.x/2,
		maxY=bboxes.boxes[i].pose.position.y+bboxes.boxes[i].dimensions.y/2,
		maxZ=bboxes.boxes[i].pose.position.z+bboxes.boxes[i].dimensions.z/2;
		boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
		boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
		boxFilter.setInputCloud(cloud.makeShared());
		boxFilter.filter(filtered);
		if (filtered.size() > 20) bboxes1.boxes.push_back(bboxes.boxes[i]);
	}
}

bool cmp(jsk_recognition_msgs::BoundingBox a, jsk_recognition_msgs::BoundingBox b)
{
	float v1,v2;
	v1=a.dimensions.x*a.dimensions.y*a.dimensions.z;
	v2=b.dimensions.x*b.dimensions.y*b.dimensions.z;
	return v1>v2?1:0;
}

int filter_bbox::enclose_(int i,vector<int>& inside_boxes)
{
	float
	MinX=bboxes1.boxes[i].pose.position.x-bboxes1.boxes[i].dimensions.x/2-0.1,
	MinY=bboxes1.boxes[i].pose.position.y-bboxes1.boxes[i].dimensions.y/2-0.1,
	MinZ=bboxes1.boxes[i].pose.position.z-bboxes1.boxes[i].dimensions.z/2-0.1,
	MaxX=bboxes1.boxes[i].pose.position.x+bboxes1.boxes[i].dimensions.x/2+0.1,
	MaxY=bboxes1.boxes[i].pose.position.y+bboxes1.boxes[i].dimensions.y/2+0.1,
	MaxZ=bboxes1.boxes[i].pose.position.z+bboxes1.boxes[i].dimensions.z/2+0.1;
	for(int j=i+1;j<bboxes1.boxes.size();j++)
	{
		// if(i==j) continue;
		float
		minX=bboxes1.boxes[j].pose.position.x-bboxes1.boxes[j].dimensions.x/2,
		minY=bboxes1.boxes[j].pose.position.y-bboxes1.boxes[j].dimensions.y/2,
		minZ=bboxes1.boxes[j].pose.position.z-bboxes1.boxes[j].dimensions.z/2,
		maxX=bboxes1.boxes[j].pose.position.x+bboxes1.boxes[j].dimensions.x/2,
		maxY=bboxes1.boxes[j].pose.position.y+bboxes1.boxes[j].dimensions.y/2,
		maxZ=bboxes1.boxes[j].pose.position.z+bboxes1.boxes[j].dimensions.z/2;
		if(minX>=MinX && maxX<=MaxX && minY>=MinY && maxY<=MaxY 
			&& minZ>=MinZ && maxZ<=MaxZ)
			inside_boxes.push_back(j);
	}
	return inside_boxes.size();
}

void filter_bbox::remove_(vector<int>& inside_boxes)
{
	for(int i=0;i<inside_boxes.size();i++)
		bboxes1.boxes.erase(bboxes1.boxes.begin()+inside_boxes[i]-i);
}

int filter_bbox::get_segs(jsk_recognition_msgs::BoundingBox box)
{
	pcl::PointCloud<VPoint> filtered;

	pcl::CropBox<pcl::PointXYZRGBA> boxFilter;
	float
	minX=box.pose.position.x-box.dimensions.x/2,
	minY=box.pose.position.y-box.dimensions.y/2,
	minZ=box.pose.position.z-box.dimensions.z/2,
	maxX=box.pose.position.x+box.dimensions.x/2,
	maxY=box.pose.position.y+box.dimensions.y/2,
	maxZ=box.pose.position.z+box.dimensions.z/2;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	boxFilter.setInputCloud(cloud.makeShared());
	boxFilter.filter(filtered);
	riseg ri(filtered);
	return ri.label-1;
}

void filter_bbox::select_(int& i)
{
	vector<int> inside_boxes;
	float
	MinX=bboxes1.boxes[i].pose.position.x-bboxes1.boxes[i].dimensions.x/2-0.1,
	MinY=bboxes1.boxes[i].pose.position.y-bboxes1.boxes[i].dimensions.y/2-0.1,
	MinZ=bboxes1.boxes[i].pose.position.z-bboxes1.boxes[i].dimensions.z/2-0.1,
	MaxX=bboxes1.boxes[i].pose.position.x+bboxes1.boxes[i].dimensions.x/2+0.1,
	MaxY=bboxes1.boxes[i].pose.position.y+bboxes1.boxes[i].dimensions.y/2+0.1,
	MaxZ=bboxes1.boxes[i].pose.position.z+bboxes1.boxes[i].dimensions.z/2+0.1;
	for(int j=i+1;j<bboxes1.boxes.size();j++)
	{
		// if(i==j) continue;
		float
		minX=bboxes1.boxes[j].pose.position.x-bboxes1.boxes[j].dimensions.x/2,
		minY=bboxes1.boxes[j].pose.position.y-bboxes1.boxes[j].dimensions.y/2,
		minZ=bboxes1.boxes[j].pose.position.z-bboxes1.boxes[j].dimensions.z/2,
		maxX=bboxes1.boxes[j].pose.position.x+bboxes1.boxes[j].dimensions.x/2,
		maxY=bboxes1.boxes[j].pose.position.y+bboxes1.boxes[j].dimensions.y/2,
		maxZ=bboxes1.boxes[j].pose.position.z+bboxes1.boxes[j].dimensions.z/2;
		if(minX>=MinX && maxX<=MaxX && minY>=MinY && maxY<=MaxY 
			&& minZ>=MinZ && maxZ<=MaxZ)
		{
			select_(j);
			inside_boxes.push_back(j);
		}
	}
	switch(inside_boxes.size())
	{
		case(0): 
			// result.boxes.push_back(bboxes1.boxes[i]);
			break;
		case(1):
			// result.boxes.push_back(bboxes1.boxes[i]);
			remove_(inside_boxes);
			break;
		case(2):
			if(get_segs(bboxes1.boxes[inside_boxes[0]])+
				get_segs(bboxes1.boxes[inside_boxes[1]])
				<= get_segs(bboxes1.boxes[i]))
			{
				// result.boxes.push_back(bboxes1.boxes[inside_boxes[0]]);
				// result.boxes.push_back(bboxes1.boxes[inside_boxes[1]]);
				// remove_(inside_boxes);
				bboxes1.boxes.erase(bboxes1.boxes.begin()+i);
				i--;
			}
			else
			{
				remove_(inside_boxes);
				break;
			}
		// case(3):
		// 	if(get_segs(bboxes1.boxes[inside_boxes[0]])+
		// 		get_segs(bboxes1.boxes[inside_boxes[1]])+
		// 		get_segs(bboxes1.boxes[inside_boxes[2]])
		// 		<= get_segs(bboxes1.boxes[i]))
		// 	{
		// 		// result.boxes.push_back(bboxes1.boxes[inside_boxes[0]]);
		// 		// result.boxes.push_back(bboxes1.boxes[inside_boxes[1]]);
		// 		// remove_(inside_boxes);
		// 		bboxes1.boxes.erase(bboxes1.boxes.begin()+i);
		// 		i--;
		// 		break;
		// 	}
		default:
			// result.boxes.push_back(bboxes1.boxes[i]);
			remove_(inside_boxes);
	}
}

void filter_bbox::callback_()
{
	if (bboxes_flag&&bboxes1_flag)
	{
		result.boxes.clear();
		filter_small_boxes_();
		sort(bboxes1.boxes.begin(),bboxes1.boxes.end(),cmp);
		int i;
		for (i=0;i<bboxes1.boxes.size();i++)
		{
			// jsk_recognition_msgs::BoundingBox cache=bboxes1.boxes[i];
			select_(i);
		}
		bboxes_pub_.publish(bboxes1);
		bboxes_flag=false;bboxes1_flag=false;
	}
}

void filter_bbox::save_bbox_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg)
{
	bboxes.boxes.clear();
	bboxes=bboxes_in_msg;
	bboxes_flag=true;
	callback_();
}

void filter_bbox::save_bbox1_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg)
{
	bboxes1.boxes.clear();
	bboxes1=bboxes_in_msg;
	bboxes1_flag=true;
	callback_();
}

void filter_bbox::save_pcd_(sensor_msgs::PointCloud2 cloud_in_msg)
{
	pcl::fromROSMsg(cloud_in_msg, cloud);
}

int main(int argc, char *argv[])
{
  	ros::init(argc, argv, "filter_bbox");
  	filter_bbox filter;
  	ros::spin();

	return 0;
}
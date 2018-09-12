#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>

using namespace cv;
using namespace std;

class label_pcd
{
public:
	label_pcd();
private:
	void save_bbox_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg);
	void save_bbox1_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg);
	void save_yolo_box_(darknet_ros_msgs::BoundingBoxes);
	void callback_();
	void add_text_(string label);
	float IOU_(darknet_ros_msgs::BoundingBox a, jsk_recognition_msgs::BoundingBox b);
	float* proj_(float *points);

	jsk_recognition_msgs::BoundingBoxArray bboxes;
	jsk_recognition_msgs::BoundingBoxArray bboxes1;
	jsk_recognition_msgs::BoundingBoxArray result;
	darknet_ros_msgs::BoundingBoxes yolo_boxes;
	visualization_msgs::MarkerArray text_;
	bool bboxes1_flag=false,bboxes_flag=false,yolo_box_flag=false;
	ros::Subscriber bboxes_sub_;
	ros::Subscriber bboxes1_sub_;
	ros::Subscriber yolo_box_sub_;
	ros::Publisher bboxes_pub_;
	ros::Publisher text_pub_;
	ros::NodeHandle nh_;
};

label_pcd::label_pcd():nh_("~")
{
	bboxes_sub_=nh_.subscribe("/bboxes",2,&label_pcd::save_bbox_,this);
	bboxes1_sub_=nh_.subscribe("/bboxes1",2,&label_pcd::save_bbox1_,this);
	yolo_box_sub_=nh_.subscribe("/darknet_ros/bounding_boxes",2,&label_pcd::save_yolo_box_,this);
	bboxes_pub_=nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bboxes_filtered",2,true);
	text_pub_=nh_.advertise<visualization_msgs::MarkerArray>("/label",2,true);
}

float label_pcd::IOU_(darknet_ros_msgs::BoundingBox a, jsk_recognition_msgs::BoundingBox b)
{
	Rect yolo_rect=Rect(Point(a.xmin,a.ymin),Point(a.xmax,a.ymax));
	float tlf[3]={b.pose.position.x-b.dimensions.x/2,b.pose.position.y+b.dimensions.y/2,b.pose.position.z+b.dimensions.z/2},
	tlb[3]={b.pose.position.x+b.dimensions.x/2,b.pose.position.y+b.dimensions.y/2,b.pose.position.z+b.dimensions.z/2},
	trf[3]={b.pose.position.x-b.dimensions.x/2,b.pose.position.y-b.dimensions.y/2,b.pose.position.z+b.dimensions.z/2},
	trb[3]={b.pose.position.x+b.dimensions.x/2,b.pose.position.y-b.dimensions.y/2,b.pose.position.z+b.dimensions.z/2},
	blf[3]={b.pose.position.x-b.dimensions.x/2,b.pose.position.y+b.dimensions.y/2,b.pose.position.z-b.dimensions.z/2},
	blb[3]={b.pose.position.x+b.dimensions.x/2,b.pose.position.y+b.dimensions.y/2,b.pose.position.z-b.dimensions.z/2},
	brf[3]={b.pose.position.x-b.dimensions.x/2,b.pose.position.y-b.dimensions.y/2,b.pose.position.z-b.dimensions.z/2},
	brb[3]={b.pose.position.x+b.dimensions.x/2,b.pose.position.y-b.dimensions.y/2,b.pose.position.z-b.dimensions.z/2};
	float *tlf2d,*trf2d,*tlb2d,*trb2d,*blf2d,*brf2d,*blb2d,*brb2d;
	tlf2d=proj_(tlf);trf2d=proj_(trf);blf2d=proj_(blf);brf2d=proj_(brf);
	tlb2d=proj_(tlb);trb2d=proj_(trb);blb2d=proj_(blb);brb2d=proj_(brb);
	int top=min(tlf2d[0],min(tlb2d[0],min(trf2d[0],trb2d[0]))),
	bot=max(blf2d[0],max(blb2d[0],max(brf2d[0],brb2d[0]))),
	left=min(tlf2d[1],min(tlb2d[1],min(blf2d[1],blb2d[1]))),
	right=max(trf2d[1],max(trb2d[1],max(brf2d[1],brb2d[1])));
	Rect delphi_rect=Rect(Point(left,top), Point(right,bot));
	Rect inter=delphi_rect&yolo_rect;
	return 1.0*inter.area()/(delphi_rect.area()+yolo_rect.area()-inter.area());
}

float* label_pcd::proj_(float *points){
  const float transmat[3][4]={
    {609.6954091642638, -721.4215973249585, -001.2512585456598, -167.8990857473499},
    {180.3842015882277,  007.6447980192031, -719.6514740348144, -101.2330669741849},
    {000.9999453885620,  000.0001243653784,  000.0104513029957, -000.2721327964059}};
  float twod[3]={0};
  for(int i=0;i<3;i++){
    for(int j=0;j<3;j++)
      twod[i]+=transmat[i][j]*points[j];
    twod[i]+=transmat[i][3];
  }
  for(int i=0;i<3;i++){
    twod[i]/=twod[2];
    twod[i]-=1;
  }
  return twod;
}

void label_pcd::add_text_(string label)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = result.boxes[result.boxes.size()-1].pose.position.x;
	marker.pose.position.y = result.boxes[result.boxes.size()-1].pose.position.y;
	marker.pose.position.z = result.boxes[result.boxes.size()-1].pose.position.z;
	marker.text=label;
	marker.scale.x = 1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	text_.markers.push_back(marker);
}

void label_pcd::callback_()
{
	if(bboxes1_flag&&bboxes_flag&&yolo_box_flag)
	{
		text_.markers.clear();
		bboxes.boxes.insert( bboxes.boxes.end(), bboxes1.boxes.begin(), bboxes1.boxes.end() );
		for (int i=0;i<yolo_boxes.bounding_boxes.size();i++)
		{
			float max_iou=0;
			int box_i;
			for(int j=0;j<bboxes.boxes.size();j++)
			{
				float iou=IOU_(yolo_boxes.bounding_boxes[i],bboxes.boxes[j]);
				if(iou>max_iou)
				{
					max_iou=iou;
					box_i=j;
				}
			}
			result.boxes.push_back(bboxes.boxes[box_i]);
			add_text_(yolo_boxes.bounding_boxes[box_i].Class);
			bboxes.boxes[box_i]=*(bboxes.boxes.end()-1);
			bboxes.boxes.erase(bboxes.boxes.end()-1);
		}
		yolo_box_flag=bboxes1_flag=bboxes_flag=false;
		result.header.frame_id="velodyne";
		bboxes_pub_.publish(result);
		text_pub_.publish(text_);
	}
}

void label_pcd::save_bbox1_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg)
{
	bboxes1.boxes.clear();
	bboxes1=bboxes_in_msg;
	bboxes1_flag=true;
	callback_();
}

void label_pcd::save_bbox_(jsk_recognition_msgs::BoundingBoxArray bboxes_in_msg)
{
	bboxes.boxes.clear();
	bboxes=bboxes_in_msg;
	bboxes_flag=true;
	callback_();
}

void label_pcd::save_yolo_box_(darknet_ros_msgs::BoundingBoxes bboxes_in_msg)
{
	yolo_boxes.bounding_boxes.clear();
	yolo_boxes=bboxes_in_msg;
	yolo_box_flag=true;
	callback_();
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"label_pcd");
	label_pcd a;
	ros::spin();
	return 0;
}
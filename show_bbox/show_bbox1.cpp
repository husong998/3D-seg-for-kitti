#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#define TH_UP   15
#define TH_DOWN 0.2

struct PointXYZIRL
{
  PCL_ADD_POINT4D;                    // quad-word XYZ
  float    intensity;                 ///< laser intensity reading
  uint16_t ring;                      ///< laser ring number
  uint16_t label;                     ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define SLRPointXYZIRL PointXYZIRL

bool point_cmp(SLRPointXYZIRL a, SLRPointXYZIRL b)
{
	return a.label<b.label;
}

bool isvalid(jsk_recognition_msgs::BoundingBox box)
{
	if (box.dimensions.x > TH_UP || box.dimensions.y > TH_UP || 
		box.dimensions.z > TH_UP)
		return false;
	if (box.dimensions.x < TH_DOWN || box.dimensions.y < TH_DOWN || 
		box.dimensions.z < TH_DOWN)
		return false;
	return true;
}

class pub_bbox
{
public:
	pub_bbox();
private:
	ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher bbox_pub_;
    void callback_(sensor_msgs::PointCloud2 cloud_in_msg);
};

pub_bbox::pub_bbox():nh_("~")
{
	sub_ = nh_.subscribe("/slr1", 110, &pub_bbox::callback_, this);
	bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/bboxes1", 110);
}

void pub_bbox::callback_(sensor_msgs::PointCloud2 cloud_in_msg)
{
    pcl::PointCloud<SLRPointXYZIRL> cloud_in_pcl;
    pcl::fromROSMsg(cloud_in_msg, cloud_in_pcl);
    sort(cloud_in_pcl.begin(), cloud_in_pcl.end(), point_cmp);

    jsk_recognition_msgs::BoundingBox current_bbox;
    jsk_recognition_msgs::BoundingBoxArray bboxes_msg;
    SLRPointXYZIRL current_point;
    current_bbox.label = 0;
    float maxx, maxy, maxz, minx, miny, minz;

    for (int i=0; i < cloud_in_pcl.size(); i++)
    {
    	current_point = cloud_in_pcl.points[i];
    	if (current_bbox.label!=current_point.label)
    	{
    		current_bbox.pose.position.x = (maxx + minx)/2;
    		current_bbox.pose.position.y = (maxy + miny)/2;
    		current_bbox.pose.position.z = (maxz + minz)/2;
    		current_bbox.dimensions.x = maxx - minx;
    		current_bbox.dimensions.y = maxy - miny;
    		current_bbox.dimensions.z = maxz - minz;
    		current_bbox.header.frame_id = "/velodyne";
    		if (isvalid(current_bbox))
    		bboxes_msg.boxes.push_back(current_bbox);

    		current_bbox.label=current_point.label;
    		// current_bbox.pose.position.x = 0;
    		// current_bbox.pose.position.y = 0;
    		// current_bbox.pose.position.z = 0;
    		maxx = current_point.x;
    		minx = current_point.x;
    		maxy = current_point.y;
    		miny = current_point.y;
    		maxz = current_point.z;
    		minz = current_point.z;
    	}
    	// current_bbox.pose.position.x += current_point.x;
    	// current_bbox.pose.position.y += current_point.y;
    	// current_bbox.pose.position.z += current_point.z;
    	if (current_point.x > maxx) maxx = current_point.x;
    	if (current_point.y > maxy) maxy = current_point.y;
    	if (current_point.z > maxz) maxz = current_point.z;
    	if (current_point.x < minx) minx = current_point.x;
    	if (current_point.y < miny) miny = current_point.y;
    	if (current_point.z < minz) minz = current_point.z;
    }

    bboxes_msg.header.frame_id = "/velodyne";
    bbox_pub_.publish(bboxes_msg);
}

int main(int argc, char *argv[])
{
  	ros::init(argc, argv, "show_bbox1");
  	pub_bbox clusters;
  	ros::spin();
	return 0;
}
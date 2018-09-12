#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cstring>
#include <dirent.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>
#define VPoint velodyne_pointcloud::PointXYZIR

constexpr long double operator"" _deg ( long double deg )
{
      return deg*3.141592/180;
}

class publish_kitti_pcd
{
public:
	publish_kitti_pcd();
  void load_from_folder(const char* dir);
  void load_from_file(std_msgs::String fp);

private:
  uint16_t find_ring_(float x, float y, float z);
  void publish_pcd_(const char *fp);

  bool single_file_mode_ = true;
  std::vector<float> col_angles_;

  ros::NodeHandle nh_;
  // ros::Publisher fp_pub_;
  ros::Publisher cloud_pub_;
  ros::Subscriber fp_sub_;
};

publish_kitti_pcd::publish_kitti_pcd():nh_("~")
{
  ROS_INFO("Initializing KITTI Velodyene reader...");

  int i;
  float start = 2.0_deg, step = -0.42539683_deg;
  for (i = 0; i < 64; i++)
    col_angles_.push_back(start + i*step);

  // fp_pub_ = nh_.advertise<std_msgs::String>("/velodyne_file_path", 110, true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 110, true);
  fp_sub_ = nh_.subscribe("/pcd_file", 110, &publish_kitti_pcd::load_from_file, this);
}

uint16_t publish_kitti_pcd::find_ring_(float x, float y, float z)
{
  uint16_t found=0;
  uint16_t top=31;
  float dist = sqrt(x*x+y*y+z*z);
  float angle = asin(z/dist);
  found=col_angles_.rend()-upper_bound(col_angles_.rbegin(),col_angles_.rend(),angle);
  if (found==0)
    return found;
  if (found==col_angles_.size())
    return found-1;
  float diff_next = fabs(col_angles_[found]-angle),
        diff_prev = fabs(angle-col_angles_[found-1]);
  return diff_next < diff_prev ? found : found-1;
}

// void publish_kitti_pcd::load_from_folder(const char* dir)
// {
//   ROS_INFO("Publishing data from folder: %s", dir);

//   std::vector<std::string> file_names;

//   char file_path[256], base_dir_path[256];
//   strcpy(base_dir_path, dir);
//   if(base_dir_path[strlen(base_dir_path)-1] != '/')
//     strcat(base_dir_path, "/");

//   DIR *base_dir = opendir(base_dir_path);
//   struct dirent *ent;
//   if (base_dir == NULL)
//   {
//     ROS_ERROR("Cannot open directory");
//     return;
//   }
//   while ((ent=readdir(base_dir)) != NULL && ros::ok())
//   {
//     if (ent->d_name[0] != '0') continue;
//     strcpy(file_path, base_dir_path);
//     strcat(file_path, ent->d_name);
//     file_names.push_back(file_path);
//   }
//   std::sort(file_names.begin(), file_names.end());
//   int fn_idx=0;

//   cv::namedWindow( "Key_event_handler", cv::WINDOW_AUTOSIZE );
//   cv::Mat1f Key_event_handler(100,100);
//   cv::imshow("Key_event_handler", Key_event_handler);

//   while (1)
//   {
//     int key=-1;
//     if (fn_idx >= file_names.size() || fn_idx < 0) fn_idx = 0;
//     publish_kitti_pcd::load_from_file(file_names[fn_idx].c_str());
//     ros::spinOnce();
//     while ( -1 == (key = cv::waitKey(1)) );
//     if (key == 13) fn_idx++;
//     else if (key == 8) fn_idx--;
//     else if (key == 27) break;
//   }
// }

void publish_kitti_pcd::load_from_file(std_msgs::String fp)
{
  ROS_INFO("Publishing data from file: %s", fp.data.c_str());
  // std_msgs::String fp_msg;
  // fp_msg.data = fp;
  // fp_pub_.publish(fp_msg);
  publish_kitti_pcd::publish_pcd_(fp.data.c_str());
}

void publish_kitti_pcd::publish_pcd_(const char* fp)
{
  ROS_INFO("Entering PCD Publisher");
  // allocate 4 MB buffer (only ~130*4*4 KB are needed)
  int32_t num = 1000000;
  float *data = (float*)malloc(num*sizeof(float));

  // pointers
  float *px = data+0;
  float *py = data+1;
  float *pz = data+2;
  float *pr = data+3;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // load point cloud
  FILE *stream;
  stream = fopen (fp,"rb");
  num = fread(data,sizeof(float),num,stream)/4;
  cloud->height=1;
  for (int32_t i=0; i<num; i++) {
    pcl::PointXYZI current_pt;
    current_pt.x = *px;
    current_pt.y = *py;
    current_pt.z = *pz;
    current_pt.intensity = *pr;
    // current_pt.ring = publish_kitti_pcd::find_ring_(*px, *py, *pz);
    cloud->points.push_back(current_pt);
    px+=4; py+=4; pz+=4; pr+=4;
    cloud->width++;
  }
  fclose(stream);

  clock_t start = clock(), end;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloudFiltered);

  end = clock();

  pcl::PointCloud<VPoint> cloudWithRing;
  for(int i = 0; i < cloudFiltered->size(); i++)
  {
    VPoint pt;
    pt.x = cloudFiltered->points[i].x;
    pt.y = cloudFiltered->points[i].y;
    pt.z = cloudFiltered->points[i].z;
    pt.intensity = cloudFiltered->points[i].intensity;
    pt.ring = find_ring_(pt.x, pt.y, pt.z);
    cloudWithRing.push_back(pt);
  }

  float runTime = ((float) (end - start)) / CLOCKS_PER_SEC;

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloudWithRing, cloud_msg);
  cloud_msg.header.frame_id = "velodyne";
  cloud_pub_.publish(cloud_msg);
  ROS_INFO("Original cloud size: %d", cloud->size());
  ROS_INFO("Filtered cloud size: %d", cloudFiltered->size());
  ROS_INFO("Filtering time: %f", runTime);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publish_kitti");
  // for (int i=0; i < 4; i++)
    // ROS_INFO("%s ", argv[1]);
  // if (argc < 3) 
  // {
  //   ROS_ERROR("Please specify file path!");
  //   return 0;
  // }
  publish_kitti_pcd pcd;
  // if (strcmp(argv[1],"-d") == 0)
  //   pcd.load_from_folder(argv[2]);
  // else if (strcmp(argv[1],"-f") == 0)
  // {
  //   pcd.load_from_file(argv[2]);
  //   ros::spin();
  // }
  // else
  // {
  //   ROS_ERROR("Unknown argument");
  //   return 0;
  // }
  ros::spin();
	return 0;
}
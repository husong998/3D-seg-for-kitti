#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstring>
#include <dirent.h>

#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc,argv,"folder_publisher");
  ros::NodeHandle nh_;
  ros::Publisher pcd_pub=nh_.advertise<std_msgs::String>("pcd_file", 2, true);
  ros::Publisher img_pub=nh_.advertise<std_msgs::String>("img_file", 2, false);
  ROS_INFO("Publishing data from folder: %s", argv[1]);

  std::vector<std::string> pcd_file_names;
  std::vector<std::string> img_file_names;

  char img_file_path[256],pcd_file_path[256],base_dir_path[256],
  img_dir_path[256],pcd_dir_path[256];
  strcpy(base_dir_path, argv[1]);
  if(base_dir_path[strlen(base_dir_path)-1] != '/')
    strcat(base_dir_path, "/");
  strcpy(img_dir_path,base_dir_path);
  strcat(img_dir_path,"image_00/data/");

  strcpy(pcd_dir_path,base_dir_path);
  strcat(pcd_dir_path,"velodyne_points/data/");

  DIR *dir = opendir(pcd_dir_path);
  struct dirent *ent;
  if (dir == NULL)
  {
    ROS_ERROR("Cannot open directory");
    return 0;
  }
  while ((ent=readdir(dir)) != NULL && ros::ok())
  {
    if (ent->d_name[0] != '0') continue;
    strcpy(pcd_file_path, pcd_dir_path);
    strcat(pcd_file_path, ent->d_name);
    pcd_file_names.push_back(pcd_file_path);
  }
  std::sort(pcd_file_names.begin(), pcd_file_names.end());
  dir = opendir(img_dir_path);
  if (dir == NULL)
  {
    ROS_ERROR("Cannot open directory");
    return 0;
  }
  while ((ent=readdir(dir)) != NULL && ros::ok())
  {
    if (ent->d_name[0] != '0') continue;
    strcpy(img_file_path, img_dir_path);
    strcat(img_file_path, ent->d_name);
    img_file_names.push_back(img_file_path);
  }
  std::sort(img_file_names.begin(), img_file_names.end());
  int fn_idx=0;

  cv::namedWindow( "Key_event_handler", cv::WINDOW_AUTOSIZE );
  cv::Mat1f Key_event_handler(100,100);
  cv::imshow("Key_event_handler", Key_event_handler);

  while (1)
  {
    int key=-1;
    std_msgs::String img_msg,pcd_msg;
    if (fn_idx >= pcd_file_names.size() || fn_idx < 0) fn_idx = 0;
    img_msg.data=img_file_names[fn_idx];
    pcd_msg.data=pcd_file_names[fn_idx];
    img_pub.publish(img_msg);
    pcd_pub.publish(pcd_msg);
    ros::spinOnce();
    while ( -1 == (key = cv::waitKey(1)) );
    if (key == 13) fn_idx++;
    else if (key == 8) fn_idx--;
    else if (key == 27) break;
  }
  return 0;
}
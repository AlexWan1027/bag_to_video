#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <fstream>

ros::Publisher pub_topic;
bool first_img, read_video_normal;
long long last_img_time = 0;
int img_num, img_name_start;
cv::VideoWriter writer;
cv::VideoCapture cap;
std::string sub_img_topic, sub_odom_topic;
std::string save_video_path, save_img_path, outfile_path;
std::string video_read_path;
int img_video_mode, update_hz;

Eigen::Quaterniond car_world_q;
Eigen::Vector3d car_world_t;

cv::Mat input_frame;
std::ofstream outfile;

void on_mouse(int event, int x, int y, int flags, void* userdata);
void video_to_img();

void imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    long long dt, now_time;
    std::cout << "fuck" << std::endl;
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    input_frame = ptr->image;
//     cv::resize(input_frame, input_frame, cv::Size(640, 480));
    
    switch(img_video_mode)
    {
      case 0:
	  if(first_img)
	  {
	      int image_width, image_hight;
	      image_hight = input_frame.rows;
	      image_width = input_frame.cols;
	      writer = cv::VideoWriter(save_video_path, CV_FOURCC('M', 'J', 'P', 'G'), 
			      30, cv::Size(image_width, image_hight));
	      first_img = false;
	  }
	  else
	  {
	      writer << input_frame;
	      cv::imshow("src_img", input_frame);
	      cv::waitKey(1);
	  }
	  break; 
      case 1:
	  cv::imshow("src_img", input_frame);
	  cv::setMouseCallback("src_img", on_mouse, 0);
	  cv::waitKey(1);
	  break;
	  
      default : 
	  throw std::string("error mode");
    }
    
    now_time = ptr->header.stamp.toNSec();
    dt = now_time - last_img_time;
    std::cout << "last_img_time: " << last_img_time << std::endl;
    std::cout << "now_time: " << now_time << std::endl;
    std::cout << "dt: " << dt << std::endl;
    last_img_time = now_time;
}

void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    car_world_q.w() = odom_msg->pose.pose.orientation.w;
    car_world_q.x() = odom_msg->pose.pose.orientation.x;
    car_world_q.y() = odom_msg->pose.pose.orientation.y;
    car_world_q.z() = odom_msg->pose.pose.orientation.z;
    
    car_world_t << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 
		   odom_msg->pose.pose.position.z;
}

void get_ros_param(ros::NodeHandle &nh_param)
{
    if(!nh_param.getParam("update_hz", update_hz))  update_hz = 50;
    if(!nh_param.getParam("img_name_start", img_name_start))  img_name_start = 0;
    if(!nh_param.getParam("img_video_mode", img_video_mode))  img_video_mode = 1;
    if(!nh_param.getParam("sub_img_topic", sub_img_topic))  sub_img_topic = "/usb_cam/image_raw";
    if(!nh_param.getParam("sub_odom_topic", sub_odom_topic))  sub_odom_topic = "/autogo/localization/pose";
    if(!nh_param.getParam("save_img_path", save_img_path))  save_img_path = "/config/image/";
    if(!nh_param.getParam("save_video_path", save_video_path))  save_video_path = "/config/0.avi";
    if(!nh_param.getParam("outfile_path", outfile_path))  outfile_path = "/config/";
    if(!nh_param.getParam("video_read_path", video_read_path))  video_read_path = "/config/0.avi";
    
    std::string pkg_path = ros::package::getPath("bag_to_video");
    save_img_path = pkg_path + save_img_path; 
    save_video_path = pkg_path + save_video_path;
    outfile_path = pkg_path + outfile_path;
    video_read_path = pkg_path + video_read_path;
    
//     outfile.open(outfile_path);
//     if(!outfile.is_open())  throw std::string("cannot open the file: ") + outfile_path;
//     cv::namedWindow("src_img");
    car_world_q.w() = 0;
    car_world_q.x() = 0;
    car_world_q.y() = 0;
    car_world_q.z() = 0;
    
    car_world_t << 0, 0, 0;
    first_img = true;
    read_video_normal = true;
    img_num = img_name_start;
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag_to_video_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    get_ros_param(nh_param);
    ros::Subscriber sub_img = nh.subscribe(sub_img_topic, 100, imageCallback);
//     ros::Subscriber sub_odom = nh.subscribe(sub_odom_topic, 100, odomCallback);

    ros::Rate loop_rate(update_hz);
  
    while (ros::ok())
    {
	if (img_video_mode == 2 && read_video_normal) video_to_img();
        ros::spinOnce();
	loop_rate.sleep();
    }
    if (img_video_mode == 2) cap.release();
//     outfile.close();
    return 0;
   
}

void video_to_img()
{
    if(first_img)
    {
	cap.open(video_read_path);
	if (!cap.isOpened()) throw std::string("video cannot open");
	first_img = false;
    }
    else
    {
	cv::Mat frame_tmp;
	if(!cap.read(frame_tmp))
        {
	    read_video_normal = false;
	    cv::destroyWindow("src_img");
            return;  
        }
        input_frame = frame_tmp.clone();
	cv::imshow("src_img", input_frame);
	cv::setMouseCallback("src_img", on_mouse, 0);
	cv::waitKey(1);
    }
}

void on_mouse(int event, int x, int y, int flags, void* userdata) 
{ 
	std::string save_img_path_tmp;
	if (event == CV_EVENT_LBUTTONDOWN)
	{  
	    save_img_path_tmp = save_img_path + std::to_string(img_num) + ".jpg";
	    cv::imwrite(save_img_path_tmp, input_frame);
	    img_num++;
	    cv::imshow("src_img ", input_frame);
// 	    outfile << car_world_q.w() << " "; 
// 	    outfile << car_world_q.x() << " "; 
// 	    outfile << car_world_q.y() << " "; 
// 	    outfile << car_world_q.z() << " "; 
// 	    outfile << car_world_t(0) << " ";
// 	    outfile << car_world_t(1) << " ";
// 	    outfile << car_world_t(2);
// 	    outfile << "\n";
// 	    std::cout << "odom" << car_world_t << std::endl;
	} 
}

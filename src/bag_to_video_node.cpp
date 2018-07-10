#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

#define PUB_VIDEO 0
#define WRITER_IMG 0
#define SHOW_INFORMATION 1
#define WRITER_VIDEO 1

ros::Publisher pub_topic;
bool pub_flag;
long long last_img_time = 0;
int img_num = 0;
cv::VideoWriter writer;
std::string SUB_IMG_TOPIC, PUB_IMG_TOPIC;
std::string READ_PATH, SAVE_PATH;
int PUB_RATE;

void imageCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
    long long dt, now_time;

    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat input_frame;
    input_frame = ptr->image;
#if WRITER_VIDEO
    if(img_num == 0)
    {
       int image_width, image_hight;
       image_hight = input_frame.rows;
       image_width = input_frame.cols;
       SAVE_PATH  = SAVE_PATH + "myvideo.avi";
       writer = cv::VideoWriter(SAVE_PATH, CV_FOURCC('M', 'J', 'P', 'G'), 
				30, cv::Size(image_width, image_hight));
    }
    else
        writer << input_frame;
#endif
    
#if WRITER_IMG
    std::string img_name = "/home/administrator/files/image/" +std::to_string(img_num)+".png"; 
    cv::imwrite(img_name, input_frame);
#endif
    
    img_num++;
    
#if SHOW_INFORMATION
    cv::namedWindow("src_img", CV_WINDOW_NORMAL);
    cv::imshow("src_img", input_frame);
    cv::waitKey(1);
    now_time = ptr->header.stamp.toNSec();
    dt = now_time - last_img_time;
    std::cout << "last_img_time: " << last_img_time << std::endl;
    std::cout << "now_time: " << now_time << std::endl;
    std::cout << "dt: " << dt << std::endl;
    last_img_time = now_time;
#endif

}

void get_ros_param(ros::NodeHandle &nh_param)
{
    if(!nh_param.getParam("pub_rate", PUB_RATE))  PUB_RATE = 20;
    if(!nh_param.getParam("sub_img_topic", SUB_IMG_TOPIC))  SUB_IMG_TOPIC = "/usb_cam/image_raw";
    if(!nh_param.getParam("pub_img_topic", PUB_IMG_TOPIC))  PUB_IMG_TOPIC = "/bag_to_video/image_raw";
    if(!nh_param.getParam("video_save_path", SAVE_PATH))  SAVE_PATH = "/home/administrator/files/video/";
    if(!nh_param.getParam("video_read_path", READ_PATH))  READ_PATH = "/home/administrator/files/video/2018_11_15.avi";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "msckf_mono_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    get_ros_param(nh_param);
    ros::Subscriber sub_img = nh.subscribe(SUB_IMG_TOPIC, 100, imageCallback);
    pub_topic = nh.advertise<sensor_msgs::Image>(PUB_IMG_TOPIC, 1000);
    
#if PUB_VIDEO
    cv::VideoCapture cap(READ_PATH);
    if(!cap.isOpened())
    {
	std::cout << "cannot open video" << std::endl;
	return -1;
    }
    cv::Mat frame;
#endif
    ros::Rate loop_rate(PUB_RATE);
  
    while (ros::ok())
    {
#if PUB_VIDEO
	sensor_msgs::ImagePtr msg_img;
	cap >> frame;  
	msg_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
	pub_topic.publish(msg_img);
#endif
        ros::spinOnce();
	loop_rate.sleep();
    }
    return 0;
   
}

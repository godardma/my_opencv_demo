#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
using namespace std;
using namespace cv;
RNG rng(12345);
// VideoWriter video("outcpp.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(680,480));
int i=0;
class MinimalImageSubscriber : public rclcpp::Node {
public:
  MinimalImageSubscriber() : Node("opencv_image_publisher"), count_(0) {
    subscriber_ =
        this->create_subscription<sensor_msgs::msg::CompressedImage>("comp_image", 10, std::bind(&MinimalImageSubscriber::frame_callback, this, std::placeholders::_1));
  }
 
private:
  void frame_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {

    //RCLCPP_INFO(this->get_logger(), "Image received");
    auto frame=cv_bridge::toCvCopy(*msg.get(), "bgr8");
    // Mat_ ??
    auto src=frame->image;

    // std::cout << src.at<double>(0,0)<<std::endl;
    double angle = 90;
    cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
    rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
    rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;

    cv::Mat dst;
    cv::warpAffine(src, dst, rot, bbox.size());
    cv::Mat dst_blur;
    blur(dst,dst_blur,Size(10,10));
    cv::Mat fullImageHSV;
    cvtColor(dst_blur, fullImageHSV, CV_BGR2HSV);
    



    //Traitement de l'image HSV ici

    cv::Mat copy = dst_blur.clone();
    cv::Mat mask = cv::Mat::zeros(dst_blur.size(), dst_blur.type());

    //Define your destination image
    cv::Mat dstImage = cv::Mat::zeros(dst_blur.size(), dst_blur.type()); 
    cv::Mat dstImage_bis = cv::Mat::zeros(dst_blur.size(), dst_blur.type());    


    inRange(fullImageHSV, Scalar(150, 120, 40), Scalar(360, 360, 360), dstImage);
    inRange(fullImageHSV, Scalar(0, 120, 40), Scalar(30, 360, 360), dstImage_bis);
    dstImage=dstImage+dstImage_bis;

    int morph_size = 3; 
  
    // Create structuring element 
    Mat element = getStructuringElement( 
        MORPH_ELLIPSE, 
        Size(2 * morph_size + 1, 
             2 * morph_size + 1), 
        Point(morph_size, morph_size)); 
  
    // Closing 
    cv::Mat closedImage = cv::Mat::zeros(dst.size(), dst.type()); 
    morphologyEx(dstImage, closedImage, 
                 MORPH_OPEN, element, 
                 Point(-1, -1), 2); 

    Mat element_bis = getStructuringElement( 
        MORPH_ELLIPSE, 
        Size(2 * 4 + 1, 
             2 * 4 + 1), 
        Point(4, 4)); 
    cv::Mat openedImage = cv::Mat::zeros(dst.size(), dst.type()); 
    morphologyEx(dstImage, openedImage, 
                 MORPH_CLOSE, element_bis, 
                 Point(-1, -1), 2); 
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( openedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // Mat drawing = Mat::zeros( openedImage.size(), CV_8UC3 );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    for( size_t i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( 0, 255, 0 );
      Scalar color_red = Scalar( 255, 0, 255 );
      minEnclosingCircle( contours[i], centers[i], radius[i] );
      circle( dst, centers[i], (int)radius[i], color_red, 2 );
      
      // drawContours( dst, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }
    // imshow( "Contours", drawing );
    cv::imshow("source", dst );
    Mat destin = Mat::zeros(dst.size(), dst.type());    
    dst.copyTo(destin, openedImage);
    // if (i<300){video.write(destin);i++;}
    // else if (i==300){video.release();cout<<"FINIIIIIIIIIIIIIIII"<<endl;}
    
    cv::imshow("closed", openedImage );
    char c=(char)cv::waitKey(25);

  }
  sensor_msgs::msg::CompressedImage::SharedPtr msg_comp;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImageSubscriber>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

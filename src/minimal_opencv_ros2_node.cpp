#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using namespace std::chrono_literals;
using namespace std;
using namespace cv;



RNG rng(12345);
// VideoWriter video("ellipse_selection3.avi",VideoWriter::fourcc('M','J','P','G'),10, Size(680,480));
int i=0;


double theta_max=63.0*M_PI/180.; //63° d'ouverture objectif
double pmax=640.0;  //résolution verticale
double r=0.6/(2.0*M_PI); //périmètre de 60mm
double fac=(r*pmax)/(2.0*tan(theta_max/2.0));

// the median 
double findMedian(vector<int> a, 
                  int n) 
{ 
  
    // If size of the arr[] is even 
    if (n % 2 == 0) { 
  
        // Applying nth_element 
        // on n/2th index 
        nth_element(a.begin(), 
                    a.begin() + n / 2, 
                    a.end()); 
  
        // Applying nth_element 
        // on (n-1)/2 th index 
        nth_element(a.begin(), 
                    a.begin() + (n - 1) / 2, 
                    a.end()); 
  
        // Find the average of value at 
        // index N/2 and (N-1)/2 
        return (double)(a[(n - 1) / 2] 
                        + a[n / 2]) 
               / 2.0; 
    } 
  
    // If size of the arr[] is odd 
    else { 
  
        // Applying nth_element 
        // on n/2 
        nth_element(a.begin(), 
                    a.begin() + n / 2, 
                    a.end()); 
  
        // Value at index (N/2)th 
        // is the median 
        return (double)a[n / 2]; 
    } 
} 

Mat DoubleMatFromVec3b(Vec3b in)
{
    cv::Mat_ mat(3,1, CV_64FC1);
    mat.at <int>(0,0) = int(in [0]);
    mat.at <int>(1,0) = int(in [1]);
    mat.at <int>(2,0) = int(in [2]);

    return mat;
};

class MinimalImageSubscriber : public rclcpp::Node {
public:
  MinimalImageSubscriber() : Node("opencv_image_publisher"), count_(0) {
    namedWindow("source",WINDOW_NORMAL);
    // resizeWindow("source", 1000, 700);
    Gamma = (Mat_<double>({4.7905734e-04, 1.9332248e-04 ,3.6835161e-04, 1.9332266e-04, 1.8628560e-04, 3.1789717e-05, 3.6835123e-04 ,3.1789317e-05, 7.3987240e-04})).reshape(3);
    cout << "Gamma = " << endl << " " << Gamma << endl << endl;
    centroid = Mat_<double>({135.98308 , 33.28846, 218.4584});
    cout << "centroid = " << endl << " " << centroid << endl << endl;
    // Gamma = (Mat_<double>({2.9627589e-04 , 1.2542386e-04 , 4.0035873e-05,1.2542389e-04  ,1.6761490e-04 ,-4.0089159e-05, 4.0035862e-05 ,-4.0089169e-05 , 1.0725303e-04})).reshape(3);
    // cout << "Gamma = " << endl << " " << Gamma << endl << endl;
    // centroid = Mat_<double>({124.14576 ,  28.179985 ,200.27927});
    // cout << "centroid = " << endl << " " << centroid << endl << endl;

    subscriber_ =
        this->create_subscription<sensor_msgs::msg::CompressedImage>("comp_image", 10, std::bind(&MinimalImageSubscriber::frame_callback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("obstacle", 10);
  }

 
private:
  void frame_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = this->now();
    point_msg.header.frame_id = "map";

    auto frame=cv_bridge::toCvCopy(*msg.get(), "bgr8");
    auto src=frame->image;
    


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
    cv::Mat mask = cv::Mat::zeros(dst_blur.size(), CV_64FC1);
    vector<int> vec_i,vec_j;

    for (int i=0;i<fullImageHSV.rows;i=i+3){
      for (int j=0;j<fullImageHSV.cols;j=j+3){
        auto hsv_elem = DoubleMatFromVec3b(fullImageHSV.at<Vec3b>(i,j));
        double theta = (double) 2.0*hsv_elem.at <int>(0,0)*M_PI/180.0;
        double r = (double) hsv_elem.at<int>(0,1);
        double z = (double) hsv_elem.at<int>(0,2);
        Mat_<double> xyz = Mat_<double>({r*cos(theta)  ,r*sin(theta) , z});

        auto inter=xyz-centroid;
        auto inter2 = inter.t()*Gamma*inter;
        double boo=norm(inter2)<1;
        if (boo){
          vec_i.insert(vec_i.end(),i);
          vec_j.insert(vec_j.end(),j);
        }
        mask.at<double>(i,j)=255*boo;
        
      }
    }
    
    int morph_size = 5;
    Mat element = getStructuringElement( 
        MORPH_ELLIPSE, 
        Size(2 * morph_size + 1, 
             2 * morph_size + 1), 
        Point(morph_size, morph_size)); 
  
    // Closing 
    cv::Mat closedImage = cv::Mat::zeros(dst.size(),  CV_8UC1); 
    morphologyEx(mask, closedImage, 
                 MORPH_CLOSE, element, 
                 Point(-1, -1), 2);
    //  morphologyEx(closedImage, closedImage, 
    //              MORPH_DILATE, element, 
    //              Point(-1, -1), 2);
    Scalar color_pink = Scalar( 0,255,0 );
    if (vec_i.size()!=0){
      auto mid_i=findMedian(vec_i,vec_i.size());
      auto mid_j=findMedian(vec_j,vec_j.size());
      Point pt = Point(mid_j, mid_i);
      point_msg.point.x=mid_j-dst.cols/2;
      point_msg.point.y=mid_i-dst.rows/2;
      point_msg.point.z=0;
      publisher_->publish(point_msg);      
      circle( dst, pt, 3, color_pink, 6 );
      // cout<<"MID"<<mid_i<<" "<<mid_j<<endl;
    }
    closedImage.convertTo(closedImage, CV_8U);


    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( closedImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // Mat drawing = Mat::zeros( openedImage.size(), CV_8UC3 );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
    double p_mes_max=0.;
    for( size_t i = 0; i< contours.size(); i++ )
    {
      Scalar color = Scalar( 0, 255, 0 );
      Scalar color_pink = Scalar( 255, 0, 255 );
      minEnclosingCircle( contours[i], centers[i], radius[i] );
      circle( dst, centers[i], (int)radius[i], color_pink, 2 );
      double p_mes=(double)radius[i];
      if (p_mes>0.0001){
        if (p_mes>p_mes_max){p_mes_max=p_mes;}
        
      }
      if (p_mes_max>0){
        cout <<"----------"<<endl;
        cout<<p_mes_max<<endl;
        double L=fac/p_mes_max;
        cout<<L<<endl;
      }
      
      
      // drawContours( dst, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    }

    
    cv::imshow("source", dst );


    // // cv::imshow("mask", mask );
    cv::imshow("closedImage", closedImage );


    // Mat destin = Mat::zeros(dst.size(), dst.type());    
    // dst.copyTo(destin, closedImage);
    // if (i<150){video.write(destin);i++;cout<<i<<endl;}
    // else if (i==150){video.release();cout<<"FINIIIIIIIIIIIIIIII"<<endl;}
    
    char c=(char)cv::waitKey(25);

  }
  sensor_msgs::msg::CompressedImage::SharedPtr rmsg_comp;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  size_t count_;
  Mat_<double> Gamma;Mat_<double> centroid;
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

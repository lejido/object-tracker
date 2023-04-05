#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders::_1; 
using namespace std; 
using namespace cv; 

class ImagePublisher : public rclcpp::Node
{
   
  public:
    ImagePublisher() : Node("image_publisher"), count_(0) 
    {
      publisher_ = this -> create_publisher<std_msgs::msg::String>("camera_frame", 10); 
      timer_ = this -> create_wall_timer(500ms, std::bind(&AngleCalculator::timer_callback, this)); 
    }
    int deviceID = 2; 
    int apiID = cv::CAP_ANY; 
    VideoCapture cap;
    cap.open(deviceID, apidID); 



  private:
    void timer_callback() 
    {
      Mat frame = imread(argv[1], CV_LOAD_IMAGE_COLOR); 
      
      auto message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(); 
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_ -> publish(message); 
    }
    
}; 

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}


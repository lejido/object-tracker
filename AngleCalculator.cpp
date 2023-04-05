#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders::_1; 

class AngleCalculator : public rclcpp::Node
{
  
  public:
    AngleCalculator() : Node("angle_calculator"), count_(0) 
    {
      publisher_ = this -> create_publisher<std_msgs::msg::String>("turn_angle", 10); 
      timer_ = this -> create_wall_timer(500ms, std::bind(&AngleCalculator::timer_callback, this)); 
      subscription_ = this -> create_subscription<std_msgs::msg::String>("coordinates", 10, std::bind(&AngleCalculator::topic_callback, this, 1)); 
    }

  private:
    int bb_cx;   
    void timer_callback() 
    {
      auto message = std_msgs::msg::String(); 
      message.data =  
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_ -> publish(message); 
    }
    

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 
    size_t count_; 
    
}; 

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AngleCalculator>());
  rclcpp::shutdown();
  return 0;
}

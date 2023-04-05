#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm> 
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders::_1; 

class MovementControl : public rclcpp::Node
{
  
  public:
    MovementControl() : Node("movement_control"), count_(0) 
    {
      subscription_ = this -> create_subscription<std_msgs::msg::String>("turn_angle", 10, std::bind(&AngleCalculator::topic_callback, this, 1)); 
    }
    // establish comms with Arduino 

  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const 
    {
      string line = msg->data.c_str(); 
      stringstream ss(line); 
      vector<string> v; 
      while (getline(ss, line, ' ')) {
        v.push_back(line); 
      }
      
      float target_position = stof(line[0]); 
      

      // implement controller


      // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 
    size_t count_; 
    
}; 

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementControl>());
  rclcpp::shutdown();
  return 0;
}


#include <chrono>
#include <iostream> 
#include <functional>
#include <memory>
#include <string>
#include <algorithm> 
#include <vector> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders::_1; 

class EncoderPublisher : public rclcpp::Node
{
  
  public:
    EncoderPublisher() : Node("encoder_publisher"), count_(0) 
    {
      publisher_ = this -> create_publisher<std_msgs::msg::String>("encoder_position", 10); 
      timer_ = this -> create_wall_timer(500ms, std::bind(&AngleCalculator::timer_callback, this)); 
      // add communication with Arduino 
      
    }

  private:

    void timer_callback() 
    {
      auto message = std_msgs::msg::String(); 
      
      string line = // receive communication from Arduino 
      stringstream ss(line); 
      vector<string> encoder_information; 
      while (getline(ss, line, ' ')) {
        encoder_information.push_back(line); 
      }

      int knob_direction = stoi(encoder_information[0]); 
      float position = stof(encoder_information[1]); 

      message.data = to_string(position); 
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_ -> publish(message); 
    }
    
    
    rclcpp::TimerBase::SharedPtr timer_; 
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; 
    size_t count_; 
    
}; 

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Encoder_Publisher>());
  rclcpp::shutdown();
  return 0;
}


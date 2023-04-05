#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm> 
#include <vector>

#include "math.h"
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
    float bb_cx = 0, bb_cy = 0; // edit values, idealy NONE 
    int width = 640, height = 480; 
    float previous_angle = -1;  
    int direction = -1; 

  private:
    
    float rad2deg(float x) {
      return (float * (180 / M_PI)); 
    }

    void timer_callback() 
    {
      if ((int)bb_cx != 0 && (int)bb_cy != 0) {
        vector<float> cameraCenterPoint {width/2, height/2, 1}; 
        vector<float> objectCenterPoint = {bb_cx, bb_cy, 1}; 
          
        // check out later eigen 
        Matrix4f K; 
        K << 427.512123, 0.000000, 345.845.842737, 
               0.000000, 426.494685, 232.152991, 
               0.000000, 0.0000000, 1.000000; 

        K_inv = K.inverse();

        vector<float> xPoint = {objectCenterPoint[0], cameraCenterPoint[1], 1}; 
        vector<float> yPoint = {cameraCenterPoint[0], objectCenterPoint[1], 1}; 

        vector<float> xVector = K_inv.dot(xPoint); 
        vector<float> yVector = K_inv.dot(yPoint); 
        vector<float> cameraCenterVector = K_inv.dot(cameraCenterPoint); 
        
        xCosAng = cameraCenterVector.dot(xVector) / (cameraCenterVector.norm() * xVector.norm()); 
        yCosAng = cameraCenterVector.dot(yVector) / (cameraCenterVector.norm() * yVector.norm()); 

        float xAng = rad2deg(acos(xCosAng)); 
        float yAng = rad2deg(acos(yCosAng)); 
        
        auto message = std_msgs::msg::String(); 
        if ((int)previous_angle == -1) {
          previous_angle = 90; 
        }

        // further implementation depends on motor 
        
        // add message data, then publish

      }
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_ -> publish(message); 
    }
    

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const 
    {
      line = msg->data.c_str(); 
      stringstream ss(line); 
      vector<string> bb_coordinates; 
      while (getline(ss, line, ' ')) {
        v.push_back(line); 
      }

      bb_cx = stof(v[0]); 
      bb_cy = stof(v[1]); 
      direction = stof(v[2]);   

      cout << "Bounding box coordinates received" << endl; 
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

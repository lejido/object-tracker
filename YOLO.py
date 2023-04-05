import rclpy
from rclpy.node import Node 
import time
from std_msgs.msg import String 
from sensor_msgs.msg import Image

import numpy as np 
import cv2
from cv_bridge import CvBridge, CvBridgeError

class YOLONode(Node):

  def __init__(self):
    super().__init__('yolo_node')
    self.publisher_ = self.create_publisher(String, 'coordinates', 1) 
    timer_period = 0.5 
    self.timer = self.create_timer(timer_period, self.timer_callback) 
    self.i = 0 
    self.j = 0 
    
    self.cap = cv2.VideoCapture("/dev/video2")

    #self.subscription = self.create_subscription(Image, 'ImageTopic', self.listener_callback, 10)
    #self.subscription # to prevent unused variable warning 
    
    self.bridge = CvBridge()
    # self.img = None 
     
    self.net = cv2.dnn.readNet("/home/yoda/ros2_ws/src/tracker/tracker/settings/yolov3.weights", "/home/yoda/ros2_ws/src/tracker/tracker/settings/yolov3.cfg")
    self.classes = [] 
    with open("/home/yoda/ros2_ws/src/tracker/tracker/settings/coco.names", "r") as f:
      self.classes = [line.strip() for line in f.readlines()]

    self.layerNames = self.net.getLayerNames() 
    self.outputLayers = [self.layerNames[i-1] for i in self.net.getUnconnectedOutLayers()]

  """
  def listener_callback(self, msg):
    cv2_img = self.bridge.imgmsg_to_cv2(msg)
    
    self.img = cv2_img # check if CvBridge actually returns an image (.jpg) or another format 
    self.get_logger().info('Frame "%s" received' % str(self.j))
    self.j += 1
  """
  def timer_callback(self):
    ret, image = self.cap.read() 
    #self.img = self.cap.read() 
    height, width, channels = image.shape

    #image = self.img
    
    #height, width, channels = image.shape
    
    blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), True, crop=False)
    
    self.net.setInput(blob) 
    outs = self.net.forward(self.outputLayers) 

    class_ids, confidences, boxes = [], [] , [] 

    for out in outs: 
      for detection in out: 
        scores = detection[5:]
        class_id = np.argmax(scores)
        confidence = scores[class_id]

        if confidence > 0.3: # change to appropriate value
          x_center = int(detection[0] * width) 
          y_center = int(detection[1] * height) 

          w = int(detection[2] * width) 
          h = int(detection[3] * height) 

          x = int(x_center - w / 2) 
          y = int(y_center - h / 2) 
          
          modified_x = width - x_center 
          modified_y = y
          boxes.append([x_center,y_center,w,h])
          confidences.append(float(confidence))
          class_ids.append(class_id) 
    
    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4) 

    for i in range(len(boxes)):
      if i in indexes:
        x, y, w, h = boxes[i] 
        label = str(self.classes[class_ids[i]])
        if label=="person":
          msg = String() 
          direction = "right"
          if (x<width/2): 
            direction = "left"
          msg.data = str(x) + " " + str(y) + " " + direction
          self.publisher_.publish(msg)
          self.get_logger().info('Bounding box coordinates: "%s"' % msg.data)
          self.i += 1
    time.sleep(3)
def main(args=None):
  rclpy.init(args=args) 

  yolo_node = YOLONode() 

  rclpy.spin(yolo_node) 

  yolo_node.destroy_node()
  rclpy.shutdown() 

if __name__ == '__main__':
  main() 


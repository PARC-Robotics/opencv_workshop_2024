# Credits to https://github.com/jeffreyttc/opencv_ros2


# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

import os

class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the /left_camera/image_raw topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      '/left_camera/image_raw', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # variable for saving images
    self.count = 0

  
  
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console
    self.get_logger().info('Receiving image frames')
 
    # Convert ROS Image message to OpenCV image
    image = self.br.imgmsg_to_cv2(data, "bgr8")
    # image = self.br.imgmsg_to_cv2(data, "mono8")

    # Display image
    cv2.imshow("Robot Left Camera", image)

    # Save images as JPG files
    key = cv2.waitKey(1)
    if key == ord("s"):
      cv2.imwrite(os.path.dirname(os.path.abspath(__file__))+'/images/image'+str(self.count)+'.jpg', image)
      self.count += 1



def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
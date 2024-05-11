
# Import all the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge

import os

class ImageSubscriber(Node):
  
  def __init__(self):

    # initialize the node with a name
    super().__init__('image_subscriber')

    # create a subscriber object
    self.image_topic = '/left_camera/image_raw'
    self.subscription = self.create_subscription(
      Image,
      self.image_topic,
      self.callback,
      10
    )
    self.subscription

    # create our cv bridge object
    self.cv_br = CvBridge()

    # variable for saving images
    self.count = 0


  def callback(self, msg):

    # display a message on console
    self.get_logger().info('Receiving image frames...')

    # convert the ROS image message to an OpenCV image
    # ROS: RGB | OpenCV: BGR
    img = self.cv_br.imgmsg_to_cv2(msg, "bgr8")

    # display image
    cv.imshow("Robot left camera", img)

    # Save images as JPG files
    key = cv.waitKey(1)
    if key == ord("s"):
      cv.imwrite(os.path.dirname(os.path.abspath(__file__))+'/images/image'+str(self.count)+'.jpg', img)
      self.count += 1






def main(args=None):

  # initialize the rclpy 
  rclpy.init(args=args)

  # create an image subscriber node
  image_subscriber = ImageSubscriber()

  # spin the node so that callback is called continuously
  rclpy.spin(image_subscriber)

  # destroy the node explicitly
  image_subscriber.destroy_node()

  # shutdown the ROS client
  rclpy.shutdown()





if __name__ == '__main__':
  main()
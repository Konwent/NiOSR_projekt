#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import numpy as np

class NewPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        self.window_name = "turtlesim controller"
        self.window_size = (512, 700)
        self.cv_image = np.zeros((self.window_size[0], self.window_size[1], 3), np.uint8)
        cv2.imshow(self.window_name, self.cv_image)
        cv2.waitKey(25)
        self.x_y = None

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.2 #okres timera
        self.timer = self.create_timer(timer_period, self.timer_callback)
        cv2.setMouseCallback(self.window_name, self.get_mouse_click)
        print("Publisher live")

    def timer_callback(self):
        if self.x_y is not None:
            cmd_velocity = Twist()
            cmd_velocity.linear.y = 0.0
            velocity = 1.0
	    #Ruch gora i dol
            if self.x_y[1] < self.window_size[0] / 2:
                cmd_velocity.linear.x = velocity
            else:
                cmd_velocity.linear.x = -velocity

            self.publisher_.publish(cmd_velocity)
            self.get_logger().info(f"Publishing: {cmd_velocity.linear.x}")
            
            #Usuwanie kwadratu
            self.cv_image = np.zeros((self.window_size[0], self.window_size[1], 3), np.uint8)

            #Rysowanie kwadratu
            square_size = 50
            color = (0, 255, 0)  #kolor
            cv2.rectangle(self.cv_image, (self.x_y[0] - square_size // 2, self.x_y[1] - square_size // 2),
                          (self.x_y[0] + square_size // 2, self.x_y[1] + square_size // 2), color, -1)
            
            cv2.imshow(self.window_name, self.cv_image)
            
        cv2.waitKey(25)

    def get_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.x_y = (x, y)

def main(args=None):
    rclpy.init(args=args)
    new_publisher = NewPublisher()
    rclpy.spin(new_publisher)
    new_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


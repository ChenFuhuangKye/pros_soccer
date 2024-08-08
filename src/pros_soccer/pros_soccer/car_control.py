import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from pros_soccer.car_models import *
import orjson
import re

class car_control(Node):

    def __init__(self):
        super().__init__('car_control')
        self.get_logger().info('Node is running')
        
        # publisher to publish the car control
        self.publisher_front = self.create_publisher(
            String,
            "/car_C_front_wheel",
            10)
        
        self.publisher_rear = self.create_publisher(
            String,
            "/car_C_rear_wheel",
            10 
        )

        # subscriber to subscribe the car state
        self.subcriber_bbox = self.create_subscription(
            String,
            "/camera/image/bbox",
            self.bbox_callback,
            10
        )

        self.subcriber_scan = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        self.vel = 10
        # Initialize the car state
        self.is_face_soccer = False
        
        # Create a timer to update the car control
        self.create_timer(0.1, self.update)

    def bbox_callback(self, msg, range: int = 10, width: int = 640):        
        '''
        bbox_callback is a callback function to get the soccer ball if it is detected by the camera

        Args:
            msg (String): the message from the yolo detection
            range (int, optional): the range to detect the soccer ball. Defaults to 10.   
            width (int, optional): the width of the screen. Defaults to 640.

        Returns:
            bool : True if the soccer ball is detected, False otherwise     
        '''
        input_str = msg.data
        class_match = re.search(r'Class: (\w+),', input_str)
        bbox_numbers = re.findall(r'\d+\.\d+', input_str)

        bbox_list = [float(num) for num in bbox_numbers]

        # class = yolo detection
        # bbox = [x1, y1, x2, y2] screen coordinates
        #  x1, y1  ------------ 
        #         |            |
        #         |            |
        #         |            |
        #          ------------ x2, y2        

        result_dict = {
            'Class': class_match.group(1) if class_match else None,
            'BBox': bbox_list
        }
        if result_dict['Class'] == 'soccer' :
            bbox_list = result_dict['BBox']
            x1, y1, x2, y2 = bbox_list
            x_center = (x1 + x2) / 2            
            if width - range < x_center or x_center < width + range :
                self.is_face_soccer = True
        else:
            self.is_face_soccer = False
                    
    def scan_callback(self, msg):
        '''
        scan_callback is a callback function to get the distance from the lidar sensor

        Args:
            msg (LaserScan): the message from the lidar sensor

        Returns:
            list: the distance from the lidar sensor
            self.lader_distance =
            [front, left front, left, left back, back, right back, right, right front]
        '''
        self.lader_distance = [
            msg.ranges[0],   msg.ranges[224],  msg.ranges[449],  msg.ranges[674], 
            msg.ranges[899], msg.ranges[1124], msg.ranges[1349], msg.ranges[1574],
                          ]
                    
    def _pub_control(self, vel_r, vel_l):
        # Generate a random control signal
        control_signal_rear = {
            "type": "/car_C_rear_wheel",
            "data": dict(CarCControl(target_vel=[vel_r, vel_l])),
        }
        # Convert the control signal to a JSON string
        control_msg_rear = String()
        control_msg_rear.data = orjson.dumps(control_signal_rear).decode()
        # Publish the control signal
        self.publisher_rear.publish(control_msg_rear)

        control_signal_forward = {
            "type": "/car_C_front_wheel",
            "data": dict(CarCControl(target_vel=[vel_r, vel_l])),
        }
        # Convert the control signal to a JSON string
        control_msg_forward = String()
        control_msg_forward.data = orjson.dumps(control_signal_forward).decode()

        # Publish the control signal
        self.publisher_front.publish(control_msg_forward)
        

    def update(self):
        if self.is_face_soccer:
            self.forward()
        else:
            self.left()
        pass

    # car forward
    def forward(self):
        self._pub_control(vel_r=self.vel, vel_l=self.vel)
    # car backward
    def backward(self):
        self._pub_control(vel_r=-self.vel, vel_l=-self.vel)
    
    # car left
    def left(self):
        self._pub_control(vel_r=-self.vel, vel_l=self.vel)
    
    # car right
    def right(self):
        self._pub_control(vel_r=self.vel, vel_l=-self.vel)
    
    # car stop
    def stop(self):
        self._pub_control(vel_r=0, vel_l=0)
    
    def set_vel(self, vel):
        self.vel = vel

    
def main():
    rclpy.init()
    node = car_control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()

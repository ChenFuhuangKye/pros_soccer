import roslibpy
import orjson
import time
import re

class CarControl:
    def __init__(self):
        # Initialize the ROS client
        ros_client = roslibpy.Ros(host='localhost', port=9090)
        ros_client.run()

        # publisher to publish the car control        
        self.car_control_front_topic = roslibpy.Topic(
            ros_client, "car_C_front_wheel", "std_msgs/String")
        self.car_control_rear_topic = roslibpy.Topic(
            ros_client, "car_C_rear_wheel", "std_msgs/String")

        # subscriber to subscribe the car state
        self.car_bbox_topic = roslibpy.Topic(
            ros_client, "/camera/image/bbox", "std_msgs/String")
        
        self.car_bbox_topic.subscribe(self.bbox_callback)
        
        self.car_scan_topic = roslibpy.Topic(
            ros_client, "/scan", "sensor_msgs/LaserScan")
        self.car_scan_topic.subscribe(self.scan_callback)

        # Initialize the car state
        self.is_face_soccer = False
        self.lader_distance = []
    
    def bbox_callback(self, message):
        '''
        bbox_callback is a callback function to get the soccer ball if it is detected by the camera

        Args:
            msg (String): the message from the yolo detection
            range (int, optional): the range to detect the soccer ball. Defaults to 20.
            width (int, optional): the width of the screen. Defaults to 640.
        Returns:
            bool : True if the soccer ball is detected, False otherwise     
        '''
        width = 320
        range = 30

        input_str = message['data']
        class_match = re.search(r'Class: (\w+),', input_str)
        bbox_numbers = re.findall(r'\d+\.\d+', input_str)

        bbox_list = [float(num) for num in bbox_numbers]
        result_dict = {
                'Class': class_match.group(1) if class_match else None,
                'BBox': bbox_list
            }        
        try:
            if result_dict['Class'] == 'soccer' and len(result_dict['BBox']) == 4:                 
                x1, y1, x2, y2 = bbox_list[0], bbox_list[1], bbox_list[2], bbox_list[3]                                
                x_center = (x1 + x2) / 2                        
                if width - range < x_center < width + range:
                    self.is_face_soccer = True
                else:
                    self.is_face_soccer = False
            else:
                self.is_face_soccer = False
        except Exception as e:
            print(f"An error occurred: {e}")
            self.is_face_soccer = False
    
    def scan_callback(self, message):
        '''
        scan_callback is a callback function to get the distance from the lidar sensor

        Args:
            msg (LaserScan): the message from the lidar sensor

        Returns:
            list: the distance from the lidar sensor
            self.lader_distance =
            [front, left front, left, left back, back, right back, right, right front]
        '''
        try:            
            self.lader_distance = [
                message['ranges'][0],  message['ranges'][224],  message['ranges'][449],  message['ranges'][674],
                message['ranges'][899], message['ranges'][1124], message['ranges'][1349], message['ranges'][1574]
            ]
        except Exception as e:
            print(f"An error occurred: {e}")
            self.lader_distance = []
           
    def set_two_wheel(self, left_wheel_value, right_wheel_value):
        self.__publish_to_writer(self.__value_ratio(left_wheel_value), self.__value_ratio(right_wheel_value))
        time.sleep(0.1)
    
    def __publish_to_writer(self, left_wheel_value, right_wheel_value):
        # Generate a random control signal
        control_signal_rear = {
            'type': '/car_C_rear_wheel',
            'data': {'target_vel': [left_wheel_value, right_wheel_value]}
        }
        # Convert the control signal to a JSON string
        control_msg__rear = {"data": orjson.dumps(control_signal_rear).decode()}
        self.car_control_rear_topic.publish(roslibpy.Message(control_msg__rear))

        control_signal_front = {
            'type': '/car_C_front_wheel',
            'data': {'target_vel': [left_wheel_value, right_wheel_value]}
        }
        # Convert the control signal to a JSON string
        control_msg_front = {"data": orjson.dumps(control_signal_front).decode()}
        self.car_control_front_topic.publish(roslibpy.Message(control_msg_front))

    def __value_ratio(self, value):
        old_min, old_max = -10, 10
        new_min, new_max = -30, 30
        mapped_value = new_min + (value - old_min) * (new_max - new_min) / (
            old_max - old_min)
        return mapped_value

def main():
    car_control = CarControl()
    print(car_control.is_face_soccer)
    while (True):     
        if  car_control.is_face_soccer:             
            print(car_control.is_face_soccer)            
            pass
        else:
            car_control.set_two_wheel(-1, 1)
            pass
        pass
if __name__ == "__main__":    
    main()

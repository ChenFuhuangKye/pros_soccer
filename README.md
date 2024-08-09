# PROS_SOCCER


## 用法

1. 開啟環境
```
./docker-compose.sh
```

## 如何控制車子
1. 打開 car_control.py

2. 可以在 main 函式中改寫

### 提供的函式與參數
1. 車子前進、後退、左轉、右轉
```
# forward
car_control.set_two_wheel(1, 1)

# backward
car_control.set_two_wheel(-1, -1)

# left
car_control.set_two_wheel(-1, 1)

# right
car_control.set_two_wheel(1, -1)
```


2. 八方位距離
```
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
```

4. 是否面對足球
> 可以更改 range 大小
```
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
    bbox_list = [float(num) for num in bbox_numbers]

    # class = yolo detection
    # bbox = [x1, y1, x2, y2] screen coordinates
    #  x1, y1  ------------ 
    #         |            |
    #         |            |
    #         |            |
    #          ------------ x2, y2        

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
```
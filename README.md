# PROS_SOCCER


## 用法

1. 進入 docker  環境
```
./setup.sh 
```
2. 編譯專案
```
r         
```
3. 打開多視窗 (使用 tmux)
```
tmux       
```
4. 開啟 rosbridge：
```
b          
```
5. 打開新視窗（例如：ctrl+b+c）並運行 car_control 節點：
```
ros2 run pros_soccer car_control
```

6. 打開新視窗（例如：ctrl+b+c）並運行 soccer_node 節點：
```
ros2 run pros_yolo soccer_node
```

## 如何控制車子
1. 打開 src/pros_soccer/pros_soccer 目錄下的檔案 car_control.py

2. 可以在 update 函式中改寫

### 提供的函式
1. 車子前進、後退、左轉、右轉
```
def forward(self):
    self._pub_control(vel_r=self.vel, vel_l=self.vel)
```

2. 更改車子速度
```
def set_vel(self, vel):
    self.vel = vel
```

3. 八方位距離
```
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

self.subcriber_scan = self.create_subscription(
    LaserScan,
    "/scan",
    self.scan_callback,
    10
)
```

4. 是否面對足球
> 可以更改 range 大小，以及相片尺度
```
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
```
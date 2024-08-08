#!/bin/bash

docker run -it -p 9090:9090 -v"$(pwd)/src:/workspaces/src" kyehuang/pros_yolo:latest  /bin/bash
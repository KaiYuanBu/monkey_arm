
# monkey_arm

ROS2 Packages for Monkey Arm Development by BKY


## Installation

Python CAN Library:
```bash
pip install python-can
```

Python CAN Library (with Serial commumication):
```bash
pip install python-can[serial]
```
    
## Docker Deployment

### Windows
To build docker image:
```bash
docker build -t monkey/humble/arm .
```

To run docker image:
```bash
docker run --rm -it --name monkey-arm --user monkey -v $PWD/.:/home/monkey/arm_ws/src -e DISPLAY=host.docker.internal:0.0 -e LIBGL_ALWAYS_INDIRECT=0 monkey/humble/arm bash
```

### Linux
```bash
docker run --rm -it --name monkey-arm --user monkey -v $PWD/.:/home/monkey/arm_ws/src -e DISPLAY=$DISPLAY -e LIBGL_ALWAYS_INDIRECT=0 monkey/humble/arm bash
```

# SocketCAN:

```bash

sudo ip link set can0 up type can bitrate 500000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
candump can0

```

## Authors

- [@LocoHao](https://github.com/LocoHao)

